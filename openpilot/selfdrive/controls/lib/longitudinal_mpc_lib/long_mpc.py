#!/usr/bin/env python3
import os
import time
import numpy as np
from openpilot.cereal import log
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
# WARNING: imports outside of constants will not trigger a rebuild
from openpilot.selfdrive.modeld.constants import index_function
from openpilot.selfdrive.controls.radard import _LEAD_ACCEL_TAU

if __name__ == '__main__':  # generating code
  from acados.acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
else:
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverCython

from casadi import SX, vertcat

MODEL_NAME = 'long'
LONG_MPC_DIR = os.path.dirname(os.path.abspath(__file__))
EXPORT_DIR = os.path.join(LONG_MPC_DIR, "c_generated_code")
JSON_FILE = os.path.join(LONG_MPC_DIR, "acados_ocp_long.json")

LongitudinalPlanSource = log.LongitudinalPlan.LongitudinalPlanSource
MPC_SOURCES = (LongitudinalPlanSource.lead0, LongitudinalPlanSource.lead1)

X_DIM = 3
U_DIM = 1
# Kans: add comfort_brake / stop_distance as runtime MPC parameters.
PARAM_DIM = 8
COST_E_DIM = 5
COST_DIM = COST_E_DIM + 1
CONSTR_DIM = 4

X_EGO_OBSTACLE_COST = 5.
X_EGO_COST = 0.
V_EGO_COST = 0.
A_EGO_COST = 0.
J_EGO_COST = 5.
A_CHANGE_COST = 200.
DANGER_ZONE_COST = 100.
CRASH_DISTANCE = .25
LEAD_DANGER_FACTOR = 0.8 # 0.75
LIMIT_COST = 1e6
ACADOS_SOLVER_TYPE = 'SQP_RTI'

# Fewer timestamps don't hurt performance and lead to
# much better convergence of the MPC with low iterations
N = 12
MAX_T = 10.0
T_IDXS_LST = [index_function(idx, max_val=MAX_T, max_idx=N) for idx in range(N+1)]

T_IDXS = np.array(T_IDXS_LST)
FCW_IDXS = T_IDXS < 5.0
T_DIFFS = np.diff(T_IDXS, prepend=[0.])
COMFORT_BRAKE = 2.5
STOP_DISTANCE = 6.0
MIN_X_LEAD_FACTOR = 0.5


def get_jerk_factor(personality=log.LongitudinalPersonality.standard):
  if personality == log.LongitudinalPersonality.relaxed:
    return 1.0
  elif personality == log.LongitudinalPersonality.standard:
    return 1.0
  elif personality == log.LongitudinalPersonality.aggressive:
    return 0.5
  else:
    raise NotImplementedError("Longitudinal personality not supported")


def get_T_FOLLOW(personality=log.LongitudinalPersonality.standard):
  if personality == log.LongitudinalPersonality.relaxed:
    return 1.75
  elif personality == log.LongitudinalPersonality.standard:
    return 1.45
  elif personality == log.LongitudinalPersonality.aggressive:
    return 1.25
  else:
    raise NotImplementedError("Longitudinal personality not supported")


def get_stopped_equivalence_factor(v_lead):
  return (v_lead**2) / (2 * COMFORT_BRAKE)


# Kans: devel-compatible runtime comfort brake / stop distance,
# while retaining tizi defaults when CarrotPlanner is absent.
def get_safe_obstacle_distance(v_ego, t_follow=None, comfort_brake=COMFORT_BRAKE, stop_distance=STOP_DISTANCE):
  if t_follow is None:
    t_follow = get_T_FOLLOW()
  return (v_ego**2) / (2 * comfort_brake) + t_follow * v_ego + stop_distance


def desired_follow_distance(v_ego, v_lead, comfort_brake, stop_distance, t_follow=None):
  if t_follow is None:
    t_follow = get_T_FOLLOW()
  return get_safe_obstacle_distance(v_ego, t_follow, comfort_brake, stop_distance) - get_stopped_equivalence_factor(v_lead)


def get_carrot_float(carrot, name, default):
  if carrot is None:
    return float(default)
  try:
    value = float(getattr(carrot, name, default))
    return value if np.isfinite(value) else float(default)
  except (TypeError, ValueError):
    return float(default)


def gen_long_model():
  model = AcadosModel()
  model.name = MODEL_NAME

  # states
  x_ego, v_ego, a_ego = SX.sym('x_ego'), SX.sym('v_ego'), SX.sym('a_ego')
  model.x = vertcat(x_ego, v_ego, a_ego)

  # controls
  j_ego = SX.sym('j_ego')
  model.u = vertcat(j_ego)

  # xdot
  x_ego_dot = SX.sym('x_ego_dot')
  v_ego_dot = SX.sym('v_ego_dot')
  a_ego_dot = SX.sym('a_ego_dot')
  model.xdot = vertcat(x_ego_dot, v_ego_dot, a_ego_dot)

  # runtime parameters
  a_min = SX.sym('a_min')
  a_max = SX.sym('a_max')
  x_obstacle = SX.sym('x_obstacle')
  a_prev = SX.sym('a_prev')
  lead_t_follow = SX.sym('lead_t_follow')
  lead_danger_factor = SX.sym('lead_danger_factor')
  comfort_brake = SX.sym('comfort_brake')
  stop_distance = SX.sym('stop_distance')
  model.p = vertcat(a_min, a_max, x_obstacle, a_prev, lead_t_follow, lead_danger_factor, comfort_brake, stop_distance)

  # dynamics model
  f_expl = vertcat(v_ego, a_ego, j_ego)
  model.f_impl_expr = model.xdot - f_expl
  model.f_expl_expr = f_expl
  return model


def gen_long_ocp():
  ocp = AcadosOcp()
  ocp.model = gen_long_model()

  Tf = T_IDXS[-1]

  # set dimensions
  ocp.dims.N = N

  # set cost module
  ocp.cost.cost_type = 'NONLINEAR_LS'
  ocp.cost.cost_type_e = 'NONLINEAR_LS'

  QR = np.zeros((COST_DIM, COST_DIM))
  Q = np.zeros((COST_E_DIM, COST_E_DIM))

  ocp.cost.W = QR
  ocp.cost.W_e = Q

  x_ego, v_ego, a_ego = ocp.model.x[0], ocp.model.x[1], ocp.model.x[2]
  j_ego = ocp.model.u[0]

  a_min, a_max = ocp.model.p[0], ocp.model.p[1]
  x_obstacle = ocp.model.p[2]
  a_prev = ocp.model.p[3]
  lead_t_follow = ocp.model.p[4]
  lead_danger_factor = ocp.model.p[5]
  comfort_brake = ocp.model.p[6]
  stop_distance = ocp.model.p[7]

  ocp.cost.yref = np.zeros((COST_DIM, ))
  ocp.cost.yref_e = np.zeros((COST_E_DIM, ))

  desired_dist_comfort = get_safe_obstacle_distance(v_ego, lead_t_follow, comfort_brake, stop_distance)

  # The main cost in normal operation is how close you are to the "desired" distance
  # from an obstacle at every timestep. This obstacle can be a lead car
  # or other object. In e2e mode we can use x_position targets as a cost
  # instead.
  costs = [((x_obstacle - x_ego) - desired_dist_comfort) / (v_ego + 10.),
           x_ego,
           v_ego,
           a_ego,
           a_ego - a_prev,
           j_ego]
  ocp.model.cost_y_expr = vertcat(*costs)
  ocp.model.cost_y_expr_e = vertcat(*costs[:-1])

  # Constraints on speed, acceleration and desired distance to
  # the obstacle, which is treated as a slack constraint so it
  # behaves like an asymmetrical cost.
  constraints = vertcat(v_ego,
                        a_ego - a_min,
                        a_max - a_ego,
                        ((x_obstacle - x_ego) - lead_danger_factor * desired_dist_comfort) / (v_ego + 10.))
  ocp.model.con_h_expr = constraints

  x0 = np.zeros(X_DIM)
  ocp.constraints.x0 = x0
  ocp.parameter_values = np.array([-1.2, 1.2, 0.0, 0.0, get_T_FOLLOW(), LEAD_DANGER_FACTOR, COMFORT_BRAKE, STOP_DISTANCE])

  # We put all constraint cost weights to 0 and only set them at runtime
  cost_weights = np.zeros(CONSTR_DIM)
  ocp.cost.zl = cost_weights
  ocp.cost.Zl = cost_weights
  ocp.cost.Zu = cost_weights
  ocp.cost.zu = cost_weights

  ocp.constraints.lh = np.zeros(CONSTR_DIM)
  ocp.constraints.uh = 1e4*np.ones(CONSTR_DIM)
  ocp.constraints.idxsh = np.arange(CONSTR_DIM)

  ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
  ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
  ocp.solver_options.integrator_type = 'ERK'
  ocp.solver_options.nlp_solver_type = ACADOS_SOLVER_TYPE
  ocp.solver_options.qp_solver_cond_N = 1
  ocp.solver_options.qp_solver_iter_max = 10
  ocp.solver_options.qp_tol = 1e-3

  ocp.solver_options.tf = Tf
  ocp.solver_options.shooting_nodes = T_IDXS

  ocp.code_export_directory = EXPORT_DIR
  return ocp


class LongitudinalMpc:
  def __init__(self, dt=DT_MDL):
    self.dt = dt
    self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)
    self.reset()
    self.source = LongitudinalPlanSource.cruise

    # Kans: values published by longitudinal_planner when available.
    self.t_follow = get_T_FOLLOW()
    self.desired_distance = 0.0

    # Kans: stopped-lead hysteresis.
    self.stopped_lead_count = 0
    self.stopped_lead_active = False

  def reset(self):
    self.solver.reset()

    self.v_solution = np.zeros(N+1)
    self.a_solution = np.zeros(N+1)
    self.a_prev = np.array(self.a_solution)
    self.j_solution = np.zeros(N)
    self.yref = np.zeros((N+1, COST_DIM))
    for i in range(N):
      self.solver.cost_set(i, "yref", self.yref[i])
    self.solver.cost_set(N, "yref", self.yref[N][:COST_E_DIM])
    self.x_sol = np.zeros((N+1, X_DIM))
    self.u_sol = np.zeros((N,1))
    self.params = np.zeros((N+1, PARAM_DIM))
    for i in range(N+1):
      self.solver.set(i, 'x', np.zeros(X_DIM))

    self.last_cloudlog_t = 0
    self.crash_cnt = 0.0
    self.solution_status = 0
    self.solve_time = 0.0
    self.x0 = np.zeros(X_DIM)
    self.set_weights()

  def set_cost_weights(self, cost_weights, constraint_cost_weights):
    W = np.asfortranarray(np.diag(cost_weights))
    for i in range(N):
      W[4,4] = cost_weights[4] * np.interp(T_IDXS[i], [0.0, 1.0, 2.0], [1.0, 1.0, 0.0])
      self.solver.cost_set(i, 'W', W)
    self.solver.cost_set(N, 'W', np.copy(W[:COST_E_DIM, :COST_E_DIM]))

    Zl = np.array(constraint_cost_weights)
    for i in range(N):
      self.solver.cost_set(i, 'Zl', Zl)

  def set_weights(self, prev_accel_constraint=True, personality=log.LongitudinalPersonality.standard, jerk_factor=None, a_change_cost_starting=0.0):
    if jerk_factor is None:
      jerk_factor = get_jerk_factor(personality)

    if prev_accel_constraint:
      a_change_cost = A_CHANGE_COST
    else:
      a_change_cost = float(a_change_cost_starting)

    cost_weights = [X_EGO_OBSTACLE_COST, X_EGO_COST, V_EGO_COST, A_EGO_COST,
                    jerk_factor * a_change_cost, jerk_factor * J_EGO_COST]
    constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, DANGER_ZONE_COST]
    self.set_cost_weights(cost_weights, constraint_cost_weights)

  def set_cur_state(self, v, a):
    v_prev = self.x0[1]
    self.x0[1] = v
    self.x0[2] = a
    if abs(v_prev - v) > 2.:
      for i in range(N+1):
        self.solver.set(i, 'x', self.x0)

  @staticmethod
  def extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau):
    a_lead_traj = a_lead * np.exp(-a_lead_tau * (T_IDXS**2)/2.)
    v_lead_traj = np.clip(v_lead + np.cumsum(T_DIFFS * a_lead_traj), 0.0, 1e8)
    x_lead_traj = x_lead + np.cumsum(T_DIFFS * v_lead_traj)
    return np.column_stack((x_lead_traj, v_lead_traj))

  def process_lead(self, lead):
    v_ego = self.x0[1]
    if lead is not None and lead.present:
      x_lead = lead.dRel
      v_lead = lead.vLead
      a_lead = lead.aLeadK
      a_lead_tau = lead.aLeadTau
    else:
      x_lead = 50.0
      v_lead = v_ego + 10.0
      a_lead = 0.0
      a_lead_tau = _LEAD_ACCEL_TAU

    min_x_lead = MIN_X_LEAD_FACTOR * (v_ego + v_lead) * (v_ego - v_lead) / (-ACCEL_MIN * 2)
    x_lead = np.clip(x_lead, min_x_lead, 1e8)
    v_lead = np.clip(v_lead, 0.0, 1e8)
    a_lead = np.clip(a_lead, -10., 5.)
    return self.extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau)

  def update(self, radarstate, personality=log.LongitudinalPersonality.standard, carrot=None):
    v_ego = self.x0[1]
    a_ego = self.x0[2]

    # Kans: keep comma native lead-following parameters isolated from Carrot Navi.
    # Carrot values are used only to build Navi/traffic-stop obstacles below.
    t_follow = get_T_FOLLOW(personality)
    comfort_brake = COMFORT_BRAKE
    stop_distance = STOP_DISTANCE

    nav_t_follow = t_follow
    nav_comfort_brake = comfort_brake
    nav_stop_distance = stop_distance
    if carrot is not None:
      get_tf = getattr(carrot, "get_T_FOLLOW", None)
      nav_t_follow = get_tf(personality, v_ego, a_ego) if callable(get_tf) else t_follow
      nav_comfort_brake = max(0.1, float(getattr(carrot, "comfort_brake", COMFORT_BRAKE)))
      nav_stop_distance = max(0.0, float(getattr(carrot, "stop_distance", STOP_DISTANCE)))

    lead_xv_0 = self.process_lead(radarstate.leadOne)
    lead_xv_1 = self.process_lead(radarstate.leadTwo)

    lead_d = float(lead_xv_0[0, 0])
    lead_v = float(lead_xv_0[0, 1])

    # Kans: devel stopped-lead detection with hysteresis.
    stopped_lead_cond = (radarstate.leadOne.present and 4.0 < lead_d < 30.0 and lead_v < 1.5)
    if stopped_lead_cond:
      self.stopped_lead_count = min(self.stopped_lead_count + 1, 20)
    else:
      self.stopped_lead_count = max(self.stopped_lead_count - 1, 0)

    self.stopped_lead_active = self.stopped_lead_count >= 3
    # Kans: 앞차가 출발하면 정지차 상태 즉시 해제
    # 정지 접근 중 오검출 방지를 위해 내 차가 거의 정지한 상태에서만 해제
    if self.stopped_lead_active and radarstate.leadOne.present and v_ego < 0.5:
      if lead_v > 0.5 or radarstate.leadOne.vRel > 0.2:
        self.stopped_lead_count = 0
        self.stopped_lead_active = False

    # 정지차로 판단되면 lead speed만 0으로 간주하고,
    # 실제 정차 간격은 StopDistanceCarrot(stop_distance)가 결정하도록 함
    lead_v_for_follow = np.clip(lead_v, 0.0, 0.3) if self.stopped_lead_active else lead_v

    # Native comma lead-following distance. Carrot dynamic_t_follow must not alter lead ACC.
    self.desired_distance = desired_follow_distance(v_ego, lead_v_for_follow, comfort_brake, stop_distance, t_follow)
    self.t_follow = t_follow

    lead_0_speed_for_obstacle = np.zeros_like(lead_xv_0[:, 1]) if self.stopped_lead_active else lead_xv_0[:, 1]
    lead_0_obstacle = lead_xv_0[:, 0] + get_stopped_equivalence_factor(lead_0_speed_for_obstacle)
    lead_1_obstacle = lead_xv_1[:, 0] + get_stopped_equivalence_factor(lead_xv_1[:, 1])

    # Kans: 정지차로 판단되면 실제 정지거리가 약간 더 확보되도록, obstacle을 0.5m 가까운 쪽으로 당김
    if self.stopped_lead_active and stop_distance > 0.0:
      lead_0_obstacle = np.maximum(0.0, lead_0_obstacle - 0.5)

    stopped_lead_1_active = (
      radarstate.leadTwo.present and 4.0 < float(lead_xv_1[0, 0]) < 30.0 and
      float(lead_xv_1[0, 1]) < 1.5)
    if stopped_lead_1_active and stop_distance > 0.0:
      lead_1_obstacle = np.maximum(0.0, lead_1_obstacle - 0.5)

    # Kans: add devel-style Carrot traffic-stop obstacle without replacing tizi's native lead MPC.
    carrot_v_cruise = get_carrot_float(carrot, "v_cruise", -1.0)
    stop_x = get_carrot_float(carrot, "stop_dist", 1000.0)
    traffic_stop_adjust = get_carrot_float(carrot, "trafficStopDistanceAdjust", 0.0)
    carrot_mode_raw = getattr(carrot, "mode", "acc") if carrot is not None else "acc"
    carrot_mode = str(getattr(carrot_mode_raw, "value", carrot_mode_raw))

    use_carrot_stop = carrot is not None and carrot_mode == "acc" and 0.0 <= stop_x < 999.0

    # Devel's cruise obstacle is only needed when Carrot provides a valid cruise target.
    # Otherwise keep native tizi lead-only behavior.
    if use_carrot_stop:
      v_cruise_target = carrot_v_cruise if carrot_v_cruise >= 0.0 else v_ego
      # Navi obstacle uses Carrot-specific following/brake/stop values,
      # while native lead0/lead1 and MPC safety parameters remain comma defaults.
      cruise_obstacle = (
        T_IDXS * np.clip(v_cruise_target, 0.0, 1e3)
        + get_safe_obstacle_distance(np.full(N + 1, v_cruise_target), nav_t_follow, nav_comfort_brake, nav_stop_distance)
      )

      adjust_dist = -2.0 if v_ego <= 0.1 else float(np.clip(traffic_stop_adjust, -2.0, 0.0))

      # Same minimum-distance guard used by devel before accepting the traffic stop target.
      d_min = np.interp(v_ego, [0.0, 10.0, 15.0, 20.0], [5.0, 45.0, 65.0, 75.0])
      if d_min < stop_x + adjust_dist < cruise_obstacle[0]:
        stop_x = cruise_obstacle[0] - adjust_dist

      traffic_stop_obstacle = stop_x * np.ones(N + 1) + adjust_dist
      x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle, traffic_stop_obstacle])

      source_idx = int(np.argmin(x_obstacles[0]))
      if source_idx == 0:
        self.source = LongitudinalPlanSource.lead0
      elif source_idx == 1:
        self.source = LongitudinalPlanSource.lead1
      else:
        self.source = LongitudinalPlanSource.cruise
    else:
      x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle])
      self.source = MPC_SOURCES[np.argmin(x_obstacles[0])]

    self.yref[:,:] = 0.0
    for i in range(N):
      self.solver.set(i, "yref", self.yref[i])
    self.solver.set(N, "yref", self.yref[N][:COST_E_DIM])

    self.params[:,0] = ACCEL_MIN
    self.params[:,1] = ACCEL_MAX
    self.params[:,2] = np.min(x_obstacles, axis=1)
    self.params[:,3] = np.copy(self.a_prev)
    self.params[:,4] = t_follow
    self.params[:,5] = LEAD_DANGER_FACTOR
    self.params[:,6] = comfort_brake
    self.params[:,7] = stop_distance

    self.run()
    if (np.any(lead_xv_0[FCW_IDXS,0] - self.x_sol[FCW_IDXS,0] < CRASH_DISTANCE) and
            radarstate.leadOne.modelProb > 0.9):
      self.crash_cnt += 1
    else:
      self.crash_cnt = 0

  def run(self):
    for i in range(N+1):
      self.solver.set(i, 'p', self.params[i])
    self.solver.constraints_set(0, "lbx", self.x0)
    self.solver.constraints_set(0, "ubx", self.x0)

    self.solution_status = self.solver.solve()
    self.solve_time = float(self.solver.get_stats('time_tot')[0])

    for i in range(N+1):
      self.x_sol[i] = self.solver.get(i, 'x')
    for i in range(N):
      self.u_sol[i] = self.solver.get(i, 'u')

    self.v_solution = self.x_sol[:,1]
    self.a_solution = self.x_sol[:,2]
    self.j_solution = self.u_sol[:,0]

    self.a_prev = np.interp(T_IDXS + self.dt, T_IDXS, self.a_solution)

    t = time.monotonic()
    if self.solution_status != 0:
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning(f"Long mpc reset, solution_status: {self.solution_status}")
      self.reset()


if __name__ == "__main__":
  ocp = gen_long_ocp()
  AcadosOcpSolver.generate(ocp, json_file=JSON_FILE)

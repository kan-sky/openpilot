#!/usr/bin/env python3
import math
import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan, should_stop
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog

A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
J_CRUISE_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MIN = -2.0 #-1.2
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py

def get_cruise_accel(e2e, v_cruise, v_ego, a_cruise_prev, angle_steers, CP, dt, accel_coast, allow_throttle):
  max_accel = ACCEL_MAX if e2e else get_max_accel(v_ego)

  if not e2e:
    a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
    a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
    a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))
    max_accel = min(max_accel, a_x_allowed)
    if not allow_throttle:
      clipped_accel_coast = max(accel_coast, ACCEL_MIN)
      coast_limit = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [max_accel, clipped_accel_coast])
      max_accel = min(max_accel, coast_limit)

  target_accel = np.clip(v_cruise - v_ego, A_CRUISE_MIN, max_accel)
  if not e2e:
    j_cruise = np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS)
    target_accel = float(np.clip(target_accel, a_cruise_prev - j_cruise * dt, a_cruise_prev + j_cruise * dt))

  return target_accel


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.a_cruise = 0.0
    self.output_a_target = 0.0
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

    # Carrot publish state. Keep a native fallback when CarrotPlanner is unavailable.
    self.v_cruise_kph = 0.0

    # Kans: grace period after leaving LongCtrlState.stopping, so
    # lead_should_stop_early doesn't immediately re-trigger a stop right as
    # we're resuming from a standstill (ego and a just-departing lead are
    # both still slow/close in that first moment by definition).
    self._long_control_state_last = LongCtrlState.off
    self._resume_grace_frames = 0

  @staticmethod
  def _enum_value(value, default=0):
    if value is None:
      return default
    return int(getattr(value, "value", value))

  def update(self, sm, carrot=None):
    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)

    # Kans: Carrot lowers the native tz cruise target here, and separately
    # (see self.mpc.update below) contributes its traffic-stop distance as a
    # third MPC obstacle alongside the two leads - clamped there against the
    # real lead car's safe-following distance so it can't override Auto
    # Resume / lead-following behavior.
    if carrot is not None:
      planner_mode = 'blended' if sm['selfdriveState'].experimentalMode else 'acc'
      carrot_target_kph = carrot.update(sm, v_cruise_kph, planner_mode)
      if carrot_target_kph >= 0.0:
        v_cruise_kph = min(v_cruise_kph, carrot_target_kph)

    self.v_cruise_kph = v_cruise_kph
    v_cruise = v_cruise_kph * CV.KPH_TO_MS

    # Kans: traffic-light stop target calculated inside CarrotPlanner.
    # Apply only as a lower cruise target; do not feed Carrot into tz MPC.
    carrot_v_cruise = getattr(carrot, 'v_cruise', -1.0) if carrot is not None else -1.0
    if carrot_v_cruise >= 0.0:
      v_cruise = min(v_cruise, carrot_v_cruise)

    if sm['controlsState'].forceDecel:
      v_cruise = 0.0

    current_long_control_state = sm['controlsState'].longControlState
    if self._long_control_state_last == LongCtrlState.stopping and current_long_control_state != LongCtrlState.stopping:
      self._resume_grace_frames = int(2.0 / self.dt)
    elif self._resume_grace_frames > 0:
      self._resume_grace_frames -= 1
    self._long_control_state_last = current_long_control_state

    long_control_off = current_long_control_state == LongCtrlState.off

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET
    reset_state = reset_state or not v_cruise_initialized

    throttle_probs = sm['modelV2'].meta.disengagePredictions.gasPressProbs
    throttle_prob = throttle_probs[1] if len(throttle_probs) > 1 else 1.0
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['vehicleParameters'].angleOffsetDeg

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.a_desired = np.clip(sm['carState'].aEgo, ACCEL_MIN, ACCEL_MAX)

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    self.mpc.update(sm['radarState'], personality=sm['selfdriveState'].personality, carrot=carrot)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Save starting point for next iteration
    a_prev = self.a_desired

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                              action_t=action_t)
    output_should_stop_mpc = should_stop(v_ego, output_a_target_mpc)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    # Kans: enter stopping state earlier only for a close stopped lead - but
    # not during the resume grace period, or this immediately re-triggers a
    # stop right as we're pulling away from a standstill (both ego and a
    # just-departing lead are still slow/close in that first moment).
    lead_one = sm['radarState'].leadOne
    lead_should_stop_early = (self._resume_grace_frames == 0 and
                              lead_one.present and lead_one.dRel < 8.0 and lead_one.vLead < 0.5 and v_ego < 0.7)

    self.a_cruise = get_cruise_accel(sm['selfdriveState'].experimentalMode, v_cruise, v_ego,
                                     self.a_cruise, steer_angle_without_offset, self.CP, self.dt,
                                     accel_coast, self.allow_throttle)
    cruise_should_stop = should_stop(v_ego, self.a_cruise)

    candidates = [(output_a_target_mpc, self.mpc.source, output_should_stop_mpc),
                  (self.a_cruise, LongitudinalPlanSource.cruise, cruise_should_stop)]
    if sm['selfdriveState'].experimentalMode:
      candidates.append((output_a_target_e2e, LongitudinalPlanSource.e2e, output_should_stop_e2e))

    output_a_target, self.mpc.source, _ = min(candidates, key=lambda c: c[0])

    # Kans: Carrot traffic-light stop uses cruise target for approach deceleration.
    # Enter LongControl stopping only near standstill.
    carrot_should_stop = (
      carrot is not None and
      self._enum_value(getattr(carrot, "xState", None), 0) in [3, 5] and
      v_ego < 0.8
    )

    self.output_should_stop = any(should_stop for _, _, should_stop in candidates) or lead_should_stop_early or carrot_should_stop
    self.output_a_target = np.clip(output_a_target, ACCEL_MIN, ACCEL_MAX)
    self.a_desired = float(self.output_a_target)
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.output_a_target + a_prev) / 2.0

  def publish(self, sm, pm, carrot=None):
    plan_send = messaging.new_message('longitudinalPlan')

    # carrotMan is optional for plan validity. Native tz services remain required.
    plan_send.valid = sm.all_checks(service_list=['carControl', 'carState', 'controlsState', 'vehicleParameters',
                                                  'radarState', 'modelV2', 'selfdriveState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.present
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    # Kans: Carrot traffic-light state feedback for cruise.py.
    longitudinalPlan.xState = self._enum_value(getattr(carrot, "xState", None), 0) if carrot is not None else 0
    longitudinalPlan.trafficState = self._enum_value(getattr(carrot, "trafficState", None), 0) if carrot is not None else 0
    longitudinalPlan.cruiseTarget = float(self.v_cruise_kph)

    # Carrot alerts (trafficStopping/trafficSignGreen/...) never reach the UI/sound
    # pipeline on their own - selfdrived only merges car_events. Cross the process
    # boundary here so selfdrived can fold them into its own Events().
    if carrot is not None:
      longitudinalPlan.events = carrot.events.to_msg()

    pm.send('longitudinalPlan', plan_send)


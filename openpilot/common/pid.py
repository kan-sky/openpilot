import numpy as np
from numbers import Number

class PIDController:
  def __init__(self, k_p, k_i, k_f=0., k_d=0., pos_limit=1e308, neg_limit=-1e308, rate=100):
    self._k_p: list[list[float]] = [[0], [k_p]] if isinstance(k_p, Number) else k_p
    self._k_i: list[list[float]] = [[0], [k_i]] if isinstance(k_i, Number) else k_i
    self._k_d: list[list[float]] = [[0], [k_d]] if isinstance(k_d, Number) else k_d

    self.k_f = float(k_f) if isinstance(k_f, Number) else k_f

    self.i_unwind_rate = 0.3 / rate
    self.i_dt = 1.0 / rate
    self.set_limits(pos_limit, neg_limit)

    self.speed = 0.0

    self.reset()

  @property
  def k_p(self):
    return np.interp(self.speed, self._k_p[0], self._k_p[1])

  @property
  def k_i(self):
    return np.interp(self.speed, self._k_i[0], self._k_i[1])

  @property
  def k_d(self):
    return np.interp(self.speed, self._k_d[0], self._k_d[1])

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.d = 0.0
    self.f = 0.0
    self.control = 0

  def set_limits(self, pos_limit, neg_limit):
    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

  def update(self, error, error_rate=0.0, speed=0.0, override=False, feedforward=0., freeze_integrator=False):
    self.speed = speed
    self.p = self.k_p * float(error)
    self.d = self.k_d * float(error_rate)
    self.f = float(feedforward) * float(self.k_f)

    if override:
      self.i -= self.i_unwind_rate * float(np.sign(self.i))

    elif not freeze_integrator:
      self.i += float(error) * self.k_i * self.i_dt

      # Clip i to prevent exceeding control limits
      control_no_i = self.p + self.d + self.f
      control_no_i = np.clip(control_no_i, self.neg_limit, self.pos_limit)

      self.i = np.clip(
        self.i,
        self.neg_limit - control_no_i,
        self.pos_limit - control_no_i,
      )
    control = self.p + self.i + self.d + self.f
    self.control = np.clip(control, self.neg_limit, self.pos_limit)

    return self.control

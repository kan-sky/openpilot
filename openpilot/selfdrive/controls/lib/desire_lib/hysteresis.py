from openpilot.common.realtime import DT_MDL

# Cap counter magnitude so a long one-sided streak can't take minutes to
# recover once the flag flips back - without this, a several-thousand-frame
# streak (e.g. a lane line legitimately invisible through a turn) leaves the
# counter deeply negative/positive and unable to cross the threshold again
# for a very long time.
EXIST_COUNTER_MAX_MAGNITUDE = 20

class ExistCounter:
  def __init__(self, sustain_sec: float = 0.2):
    self.counter = 0
    self.true_count = 0
    self.false_count = 0
    self.threshold = int(sustain_sec / DT_MDL)

  def update(self, exist_flag: bool):
    if exist_flag:
      self.true_count += 1
      self.false_count = 0
      if self.true_count >= self.threshold:
        self.counter = min(max(self.counter + 1, 1), EXIST_COUNTER_MAX_MAGNITUDE)
    else:
      self.false_count += 1
      self.true_count = 0
      if self.false_count >= self.threshold:
        self.counter = max(min(self.counter - 1, -1), -EXIST_COUNTER_MAX_MAGNITUDE)
    return self.counter

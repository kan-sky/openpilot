"""Independent, source-aged section observations carried by the HUD relay."""
from collections import deque
import math


class SectionTracker:
  TTL = 4.0

  def __init__(self):
    self.supported = False
    self.session = None
    self.sequence = -1
    self.retired = deque(maxlen=8)
    self.observation = None
    self.deadline = -math.inf

  def clear(self):
    # Keep the generation watermark: a heartbeat cannot revive revoked data.
    self.observation = None
    self.deadline = -math.inf

  def accept(self, guidance, relay_age, relay_deadline, now):
    if not isinstance(guidance, dict):
      self.clear()
      return
    if 'section' in guidance:
      self.supported = True
    section = guidance.get('section')
    if not isinstance(section, dict):
      self.clear()
      return
    session, sequence, age = (section.get(k) for k in ('session', 'sequence', 'ageMs'))
    if (type(section.get('version')) is not int or section['version'] != 1
        or not isinstance(session, str) or not 1 <= len(session) <= 64
        or type(sequence) is not int or not 0 <= sequence <= 2**53
        or type(age) is not int or not 0 <= age < self.TTL * 1000):
      self.clear()
      return
    if session in self.retired or (session == self.session and sequence < self.sequence):
      return
    if session != self.session:
      if self.session is not None:
        self.retired.append(self.session)
      self.clear()
      self.session, self.sequence = session, -1
    deadline = min(relay_deadline, now + self.TTL - age / 1000 - relay_age)
    limit, distance = section.get('limitKph'), section.get('distanceM')
    eligible = (guidance.get('active') is True and section.get('active') is True
                and section.get('suspended') is False and section.get('offRoute') is False
                and type(limit) is int and 20 <= limit <= 140
                and type(distance) is int and 0 < distance <= 100000
                and 0 <= relay_age < self.TTL and now < deadline)
    if sequence == self.sequence:
      self.deadline = min(self.deadline, deadline)
      if not eligible:
        self.clear()
      return
    self.sequence = sequence
    self.clear()
    if eligible:
      self.observation = (session, limit, distance)
      self.deadline = deadline

  def snapshot(self, now):
    if now >= self.deadline:
      self.clear()
    return self.observation

import unittest
import zlib

from snapshot import LatestState, encode, decode


class SnapshotTests(unittest.TestCase):
  def test_latest_only_replaces_unsent(self):
    slot = LatestState()
    for i in range(100):
      slot.put((i, 'state'))
    self.assertEqual(slot.take(0), (99, 'state'))
    self.assertIsNone(slot.take(0))
    self.assertEqual((slot.produced, slot.replaced), (100, 99))

  def test_taken_state_not_replaced(self):
    slot = LatestState()
    slot.put(1)
    taken = slot.take(0)
    slot.put(2)
    slot.put(3)
    self.assertEqual(taken, 1)
    self.assertEqual(slot.take(0), 3)
    self.assertEqual(slot.replaced, 1)

  def test_utf8_json_roundtrip(self):
    for compressed in (False, True):
      raw, wire = encode({'text': '\ud55c\uae00', 'points': [1.2] * 80}, compressed)
      self.assertEqual(decode(wire, len(raw), compressed), raw)
      self.assertIn('\ud55c\uae00'.encode(), raw)

  def test_bounded_decompression(self):
    wire = zlib.compress(b'A' * 40000)
    with self.assertRaises(ValueError):
      decode(wire, 32768, True)

  def test_truncated_and_trailing_rejected(self):
    raw, wire = encode({'state': 42}, True)
    for bad in (wire[:-1], wire + b'x', wire + wire):
      with self.assertRaises(ValueError):
        decode(bad, len(raw), True)

  def test_invalid_sizes(self):
    with self.assertRaises(ValueError):
      encode({'state': 'x' * 32768}, True)
    with self.assertRaises(ValueError):
      decode(b'a', 32769, False)
    with self.assertRaises(ValueError):
      decode(b'a', 2, False)


if __name__ == '__main__':
  unittest.main()

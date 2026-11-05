import io
import time
import unittest
import zlib

import client


class Reader:
  def __init__(self, data, chunk=3):
    self.data = io.BytesIO(data)
    self.chunk = chunk

  def settimeout(self, seconds):
    assert seconds > 0

  def recv(self, size):
    return self.data.read(min(size, self.chunk))


class ProtocolTests(unittest.TestCase):
  def test_cross_language_fixture(self):
    self.assertEqual(client.pattern(0, 8).hex(), '3c47d1aa629f57a3')

  def test_frame_sizes(self):
    for tx, rx in [(20, 20), (2409, 1932), (32768, 32768)]:
      frame = client.request_frame(4, tx, rx)
      self.assertEqual(len(frame), tx)
      magic, seq, sent, received, crc = client.HEADER.unpack(frame[:20])
      self.assertEqual((magic, seq, sent, received), (b'HLB1', 4, tx - 20, rx - 20))
      self.assertEqual(crc, zlib.crc32(frame[20:]))

  def test_invalid_sizes(self):
    for tx, rx in [(19, 20), (20, 19), (32769, 20), (20, 32769)]:
      with self.assertRaises(ValueError):
        client.request_frame(0, tx, rx)

  def test_sequence_bounds(self):
    for seq in [-1, 2**31]:
      with self.assertRaises(ValueError):
        client.request_frame(seq, 20, 20)

  def test_fragmented_read(self):
    self.assertEqual(client.receive(Reader(b'abcdefgh'), 8, time.monotonic() + 1), b'abcdefgh')

  def test_early_close(self):
    with self.assertRaises(ConnectionError):
      client.receive(Reader(b'ab'), 4, time.monotonic() + 1)

  def test_expired_deadline(self):
    with self.assertRaises(TimeoutError):
      client.receive(Reader(b'ab'), 2, time.monotonic() - 1)

  def test_wrong_response(self):
    for fields in [(b'BAD!', 0, 0, 0, 0), (b'HLR1', 1, 0, 0, 0),
                   (b'HLR1', 0, 32769, 0, 0), (b'HLR1', 0, 0, 1, 0)]:
      with self.assertRaises(ValueError):
        client.response_body(client.HEADER.pack(*fields), 0, 20, 20)

  def test_percentile(self):
    self.assertEqual(client.percentile([2, 1, 4, 3], .5), 2)
    self.assertEqual(client.percentile([2, 1, 4, 3], .99), 4)
    self.assertIsNone(client.percentile([], .5))

  def test_guard_failure_before_traffic(self):
    def deny():
      raise RuntimeError('not offroad')
    with self.assertRaisesRegex(RuntimeError, 'not offroad'):
      client.measure(None, seconds=1, hz=5, tx=20, rx=20, guard=deny)

  def test_disconnect_reported_not_success(self):
    class Closed(Reader):
      def sendall(self, value):
        pass
    result = client.measure(Closed(b''), seconds=.1, hz=5, tx=20, rx=20, guard=lambda: None)
    self.assertEqual(result['verified_exchanges'], 0)
    self.assertEqual(result['attempts'], 1)
    self.assertIn('ConnectionError', result['error'])

  def test_corruption_detected(self):
    class Corrupt(Reader):
      def sendall(self, value):
        pass
    data = client.HEADER.pack(b'HLR1', 0, 1, 0, zlib.crc32(b'x')) + b'x'
    result = client.measure(Corrupt(data), seconds=.1, hz=5, tx=20, rx=21, guard=lambda: None)
    self.assertEqual(result['verified_exchanges'], 0)
    self.assertIn('Corrupt response', result['error'])


if __name__ == '__main__':
  unittest.main()

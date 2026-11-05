import json
import unittest
from unittest.mock import MagicMock, call, patch

import fault_check


def connection(error=None):
  conn = MagicMock()
  conn.__enter__.return_value = conn
  conn.getsockopt.return_value = bytes([2, 0])
  conn.recv.return_value = b''
  conn.connect.side_effect = error
  return conn


class RecoveryTests(unittest.TestCase):
  def run_trial(self, sockets):
    with patch('sys.argv', ['fault_check', '--peer', '00:11:22:33:44:55', '--channel', '3']), \
         patch.object(fault_check, 'socket') as radio, \
         patch.object(fault_check, 'check_offroad'), \
         patch.object(fault_check, 'measure', return_value={'error': None, 'verified_exchanges': 25}), \
         patch.object(fault_check.time, 'sleep') as sleep, \
         patch('builtins.print') as output:
      radio.socket.side_effect = sockets
      exit_code = 0
      try:
        fault_check.main()
      except SystemExit as exc:
        exit_code = exc.code
      return exit_code, json.loads(output.call_args.args[0]), sleep.call_args_list

  def test_reset_then_bounded_recovery(self):
    rejected = connection(ConnectionResetError(104, 'reset'))
    code, result, sleeps = self.run_trial([connection(), rejected, connection()])
    self.assertEqual(code, 0)
    self.assertTrue(result['intentional_corruption_rejected'])
    self.assertIn('104', result['reconnect_attempts'][0]['error'])
    self.assertIsNone(result['reconnect_attempts'][1]['error'])
    self.assertEqual(sleeps, [call(0), call(0.5)])
    rejected.close.assert_called_once()

  def test_exhausted_retries_fail_and_retain_each_error(self):
    rejected = [connection(ConnectionRefusedError(111, 'refused')) for _ in range(4)]
    code, result, sleeps = self.run_trial([connection(), *rejected])
    self.assertEqual(code, 1)
    self.assertIsNone(result['recovery'])
    self.assertEqual(len(result['reconnect_attempts']), 4)
    self.assertTrue(all('111' in attempt['error'] for attempt in result['reconnect_attempts']))
    self.assertEqual(sleeps, [call(0), call(0.5), call(1), call(2)])
    for conn in rejected:
      conn.close.assert_called_once()


if __name__ == '__main__':
  unittest.main()

# Navdy native Bluetooth measurements, 2026-10-19

## Conclusion

Direct comma/Navdy Bluetooth has enough measured throughput for the sampled
HUD/navigation-sized traffic. The unchanged synchronous 5-Hz request/reply
pattern does NOT meet a strict 200-ms cadence, even after reducing frame sizes.
It is feasible to build a transport, not ready to replace the production USB
bridge based on this trial alone. No vehicle/control data was injected.

## Scope and target

- Actual comma running AGNOS 19.8-carrot-bt1 and WIP-FR, native WCN3990/BlueZ.
- Actual Navdy Display, Android API 22, installed HUD version 3095-9fe58de.
- User explicitly authorized pairing the identified pair. Secure RFCOMM with
  matching dedicated UUID, exact bonded-peer allowlist and link security level 2.
- Native Android server and Linux client exchange deterministic binary frames
  over Bluetooth. USB remains the management/production bridge, NOT the benchmark
  data transport. Both directions require sequence, CRC32 and full-byte equality.
- `IsOnroad=0` and `IsEngaged=0` checked before connection and each exchange.
  No force-onroad, vehicle-control change, production APK edit or device reboot.
- Current phone link remained connected at sampled points. An existing GATT
  connection remained present and original HUD/connection-service PIDs survived.
  This does not prove ambient command delivery or active phone navigation load:
  sampled USB `roadPreview` and `navigationFeedback` were null.

The optional separate test APK was rejected with
`INSTALL_FAILED_VERIFICATION_FAILURE`; the firmware explicitly disallows new
packages in user builds. Installer/verification policies were NOT changed.
The owned benchmark DEX instead ran temporarily through the already-authorized
root ADB shell using `app_process`. No test APK was installed.

## Measurements

Frame sizes include the 20-byte test header. Rates count fully verified
bidirectional exchanges. RTT includes send/receive and response verification,
but excludes connection setup and deterministic request preparation.

| Trial | Seconds | Request/response bytes | Verified/scheduled | Skipped slots | Rate | RTT median / p95 / maximum |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Initial full size | 30 | 2409 / 1932 | 135 / 150 | 15 | 4.500 Hz | 127.8 / 236.0 / 328.7 ms |
| Full size, longer run | 180 | 2409 / 1932 | 875 / 900 | 25 | 4.861 Hz | 76.9 / 168.9 / 342.7 ms |
| Initial reduced size | 60 | 1200 / 1000 | 300 / 300 | 0 | 5.000 Hz | 45.6 / 116.4 / 172.4 ms |
| Reduced size, longer run | 120 | 1200 / 1000 | 597 / 600 | 3 | 4.975 Hz | 43.5 / 125.3 / 325.9 ms |
| Stop-and-wait bulk | 30.505 actual | 8192 / 8192 | 111 / unscheduled | n/a | 3.639 Hz | 213.0 / 471.4 / 556.3 ms |

Normal trials reported no payload corruption or transport error. Skipped slots
are unsent scheduled updates after slow exchanges, NOT corrupted/dropped packets.
The 180-second full-size run had 15 RTTs over 200 ms; the 120-second reduced run
still had two. Do not use the initial 60-second result as a lasting jitter-free
guarantee. Setup times in the normal trials ranged from about 46 ms to 2.57 s.

Bulk verified aggregate application throughput was 59,616.9 B/s, about 59.6 KB/s
or 477 kbit/s. Sampled full-size 5-Hz traffic would require 21,705 B/s. This is a
useful capacity comparison, NOT the radio's ceiling or a guaranteed minimum.
Payloads, scheduling, phone activity and interference can change the outcome.

An offline example built from the existing display serializer/test fixture
(two leads, eight geometry lines, Korean warning) was 2364 bytes of JSON versus
1083 bytes using zlib level 1, with identical decompressed bytes. The reduced
radio trials used synthetic fixed-size frames; production compression was not
implemented, forwarded to the real HUD or rendered during this test.

## Reconnection and corruption

- Three ordinary disconnect/reconnect trials each completed 25/25 exchanges
  over five seconds. Connection setup was 55.6, 45.9 and 60.2 ms.
- One intentionally corrupted request was rejected with zero accepted frames.
  Immediate reconnection failed with `Connection reset by peer`. A later normal
  connection succeeded without rebooting or resetting either radio.
- A repeat with recorded bounded retry reproduced the immediate reset, then
  succeeded after a 0.5-second delay (44.5-ms connection setup). All subsequent
  25/25 exchanges verified, maximum RTT 53.1 ms. No production app restart.
- The test server's deliberate ten-minute lifetime expired between test phases.
  One subsequent connection was refused; restarting ONLY the test server and
  checking its new SDP record resumed the trial. This is not spontaneous radio
  failure and is separate from the immediate post-corruption reset.

This supports reconnect backoff rather than adapter reset. It does not establish
recovery under every disconnect, long-term stability or exact reset root cause.

## Restoration and verification

- Stopped only the identified temporary benchmark process. No remaining bench
  process or APK package; removed only this trial's temporary device inputs.
- Restored Navdy scan mode from CONNECTABLE (21) to original NONE (20), without
  toggling its radio. Existing phone connection and one GATT link still present.
- Restored comma's original radio-off policy: both services inactive, no HCI
  controller and no `/data/bluetooth/ENABLED` flag. Approved bond is retained.
- Production USB bridge still connected, extended feedback supported, status
  file age 0.15 s. Original HUD PIDs 712/735 and comma boot ID unchanged.
  Offroad vehicle/model messages were stale as expected; this is NOT fresh
  real-vehicle telemetry or optical HUD validation.
- Final post-cleanup check saw the USB sequence advance from 17237 to 17252
  in three seconds; status age remained below 0.1 s. All trial inputs/processes
  were absent, with the same original HUD processes and unchanged boot.

## Production work still required

1. Add a separate authenticated, paired-peer Bluetooth transport without changing
   the phone/ambient service or resetting the shared adapter on failure.
2. Reduce repeated geometry payloads, bound frame/decompression sizes, and keep
   only the newest display state rather than building a delayed update queue.
3. Separate periodic display transmission from reverse navigation responses;
   retain existing navigation validity, timestamps, source selection and expiry.
4. Bound reconnect retries/backoff and expire stale states. A failed link must
   not leave stale navigation/HUD information looking current.
5. Validate real changing navigation, phone plus ambient commands, USB handover,
   longer runs and rendered HUD behavior. Current trials do not cover these.

## Reproduction and evidence

Local verification: 14 benchmark/retry Python tests, eight existing Bluetooth
preflight tests, Java protocol tests and Python compilation pass. These are
separate from the native radio measurements above; no road-validation claim.

Tools: [benchmark README](../tools/navdy/bluetooth_bench/README.md).
Private raw JSON results are retained locally under
`~/Downloads/Navdy-Bluetooth-Bench-20261019-results/` (long trials, reconnects,
and `corruption-recovery.json`). Initial shorter trials are retained in the task
execution output. Generated artifacts and signing material remain outside Git.

Actually measured server DEX (r4) SHA256:
`cad91eda93745ca3bce84ccb01e32370905ffdf1512f4228119489d18ad05e6e`.
Actually measured client SHA256:
`3e9bc99d556a7d3f2077237328d6b60558cfb249ddbf4a60b320481c07138317`.
Later helper builds only add connection-state inspection and scan-mode cleanup;
the recorded campaign used the original r4 server.

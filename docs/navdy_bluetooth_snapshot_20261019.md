# Navdy latest-snapshot Bluetooth trial, 2026-10-19

## Scope

Follow-up to [the original radio benchmark](navdy_bluetooth_benchmark_20261019.md).
The user requested another real-device trial using latest-only pending state,
smaller payloads and receiver freshness enforcement. Production USB stays active.
This is a separate paired RFCOMM server with an isolated in-memory state sink;
it never sends synthetic data to the real HUD, navigation input or vehicle.

Changes are limited to the benchmark toolkit and documentation. Existing dirty
production bridge/power/Jetson changes are preserved, not deployed by this trial.

## Implemented Policy

- Producer generates full synthetic display states at 5 Hz. One unsent state
  slot is overwritten by a newer state; an in-flight frame is never truncated.
  There is still only one outstanding request/reply. No unbounded socket queue.
- Use actual UTF-8 JSON from the existing display serializer, changing sequence
  and speed each tick, with eight geometry lines, two leads and Korean text.
  Test raw JSON versus zlib level 1. Echoed reverse traffic has the same payload
  size; this is not an active phone TBT payload or an asymmetrical nav workload.
- Bound raw and compressed payloads to 32 KiB. Reject truncated, oversized,
  trailing or corrupt compressed input. Check decoded CRC, sequence and complete
  byte-for-byte roundtrip including Korean text. No lossy field reduction.
- Receiver-issued hello/ACK lease and relative monotonic capture time provide
  conservative age without relying on either device's wall clock. Calculation
  includes prior ACK transit, 2 ms and a 1000-ppm clock-rate margin. It assumes
  honest timestamps and clock rate within that margin; not a general safety proof.
- Experimental TTL is 500 ms. Expired/out-of-order states cannot refresh stored
  state. Its lifetime is the remaining TTL, not another 500 ms from receipt.
  A 100-ms watchdog clears expired storage; reads also check immediately.
- Commands, warning events and production navigation are excluded from the
  replaceable state slot. Their reliable delivery is not implemented by this test.

## Native Results

Same device pair and synthetic fixture, sequential raw/compressed/raw/compressed:

| Phase | Duration | Verified / produced | RTT median | RTT p95 | RTT maximum |
| --- | ---: | ---: | ---: | ---: | ---: |
| Raw A | 60 s | 300 / 300 | 98.0 ms | 187.4 ms | 259.4 ms |
| zlib A | 120 s | 600 / 600 | 48.6 ms | 135.8 ms | 243.1 ms |
| Raw B | 60 s | 300 / 300 | 85.5 ms | 191.4 ms | 253.8 ms |
| zlib B | 60 s | 300 / 300 | 45.5 ms | 128.9 ms | 197.1 ms |

All 1,500 normal-phase states verified and applied, about 5 Hz on average. No
normal-phase state expired, coalesced or failed validation. This is NOT perfectly
uniform 200-ms delivery: compression still had a 243.1-ms worst RTT. Maximum
capture-to-ACK age was 244.7 ms for compressed A and 201.2 ms for compressed B.
The earlier benchmark skipped schedule slots after a slow exchange; this producer
continues generating state independently. Differences from the earlier test must
not be attributed to compression alone. Current raw/compressed A/B uses the same
scheduler and payload, but is still a short sequential trial, not RF control.

Mean raw size was about 2318 bytes, compressed about 1027 bytes (55.7% reduction).
Median zlib encode time on comma was 1.76/2.25 ms, decode time on Navdy about
0.42 ms. These are offroad measurements, not onroad CPU-budget validation.

### Deliberate Fault Trials

- 20-second sender-stall phase: pause sending for 800 ms after each seven verified
  states while generation continues. Of 100 produced states, 70 were sent and
  verified, 29 unsent states were replaced, one was pending at trial end. Queue
  capacity remained one; no stale backlog replay. Accepted states' maximum
  capture-to-ACK age was 163.3 ms, conservative receiver age 186.7 ms.
- During those stalls, ACKs confirmed nine stored-state expirations. The server
  log counted ten by session close, including the final unacknowledged expiry.
  The counter proves the isolated sink expired, not an actual visual transition.
- 20-second stale-injection phase: report one-second-old capture times on every
  seventh message. All 14 injected stale states were rejected; the remaining
  86 states applied. Two previous states expired while waiting for fresh data.
  Every response still passed full payload validation, with no transport error.

The current test supports compression plus bounded latest-state queuing and
freshness expiry for display snapshots. It does not eliminate radio jitter or
implement reliable commands/events. Actual TBT, ambient commands, HUD rendering,
onroad load, long-duration operation and suspend/resume remain unverified.

## Restoration and Local Checks

The original HUD processes stayed 712/735 and the comma boot ID did not change.
The phone stayed connected at sampled checks and an existing GATT connection
remained present; command delivery/active navigation was not measured. The
production USB status advanced from sequence 22251 to 22266 over three seconds,
with status age 53-54 ms, connected and extended-feedback-capable.

Stopped only the identified benchmark process and removed its temporary inputs.
Navdy scan mode restored to NONE (20); comma radio services inactive, no HCI
controller and no enable flag. The explicitly authorized bond is preserved.
Production USB remains selected. No actual HUD Bluetooth migration was deployed.

Local checks: 20 benchmark Python tests, eight Bluetooth preflight tests and
standalone Java protocol/expiry tests. The 13 existing display-serializer tests
also pass with the available Python 3.12 environment and test-directory conftest
boundary, including real Cap'n Proto payloads. The repository-wide conftest needs
unavailable local `params_pyx`; this is not a full openpilot test-suite run.

## Evidence

Private JSON results: `~/Downloads/Navdy-Bluetooth-Snapshot-20261019-results/`.
Server DEX SHA256:
`929d1a08f1966a721555c6a13f61f3348d87e418162e61cd4b7a68e7771bb199`.
Measured `snapshot.py` SHA256:
`559bb14e80bdebc80296c77b8650065ec40d9f19f163957aaa7d7953fff73552`.
Generated fixture SHA256:
`5567f00ff9bfc7377e49b13dc318a100039abfcd31fde0c271b98065ec214687`.

No APK replacement, whole-device reboot, driving-model change, forced onroad,
phone/ambient bond deletion or road test is part of this trial.

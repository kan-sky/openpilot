# Navdy Wireless Flicker, 2026-10-21

## Scope and Evidence

The user reported intermittent loss of all Comma widgets and ambient light
while driving. Device work in this phase was passive observation only. Local
source/APK preparation does not authorize a live install while moving, even
with openpilot disengaged. Installation requires a stationary vehicle and a
fresh inactive/P/parking-brake check.

The observed Comma was running WIP-FR `501ea02b22`. The installed HUD baseline
is SHA256 `f6d5367f8cf61d7d566371e36d20c2036f148262a1f305207083f780105cce8f`.
The new HUD patch must preserve its model-navigation classes, phone behavior,
ambient lifecycle, resources, native libraries and existing signer.

Three bounded passive observations distinguish the failures:

- A 120 s status observation recorded six display clears and an 8.43 s gap
  between good replies during one reconnect. Vehicle source data was usable.
- A 125 s HCI/status observation recorded 164 clears but only one reconnect.
  For 303 complete CRC-valid decoded ACKs, request-to-ACK median was 276.95 ms,
  p99 418.37 ms, maximum 518.00 ms. Complete ACK arrival to bridge processing
  median was 1.31 ms, p99 20.14 ms. Not every HCI ACK was decoded.
- A separate 45 s observation without btmon recorded 45 clears while remaining
  in the same connected session. This is not merely a packet-monitor artifact.

The recorded disconnect followed the Comma's 650 ms response timeout. Comma
initiated RFCOMM/L2CAP teardown, followed by HCI local-host termination (0x16).
This particular trace does not show Navdy reboot or radio link supervision
failure first. It does not establish the ultimate reason for Navdy/radio
response latency or prove absence of other faults.

The old stop-and-wait protocol's conservative 500 ms display lifetime includes
the previous ACK return transit and current request transit. At this latency,
waiting for each ACK before sending again can leave an expiry gap even while
connected. The existing ambient controller separately requests OFF after
three seconds without known vehicle data. The 500 ms unknown-state clear alone
does not directly switch the ambient light off; the longer reconnect gap can.
Physical LED output was not captured.

## Correction

- Negotiate wire protocol v2 with the new HUD; retain original v1 fallback.
- Keep at most two outstanding snapshots and one partial transmit buffer.
  Capture remains at 5 Hz. Drop a busy tick, never accumulate stale frames.
- Read ACKs with one bounded nonblocking IO reader, recording receipt before
  the display tick; retain only two responses. Parse/apply on the bridge thread.
- Accept a bounded eight-lease history on the receiver so pipelined snapshots
  may use the same lease. Preserve immutable capture age and strict ordering.
- Keep display admission/expiry at 500 ms, USB priority at 750 ms and existing
  main-thread checks. No stale engagement hold or fake freshness is introduced.
- Separate the 1.5 s connection response deadline from display lifetime. An
  expired display ACK renews only the protocol lease, not the old display state.
- Consume real replies once, charge transit against navigation freshness and
  leave feedback receipt timestamps unchanged on empty polls.
- Leave ambient's three-second fail-safe, offroad sleep/door behavior, native
  power helper, navigation/model validity and actual vehicle controls unchanged.

See `tools/navdy/WIRELESS_PROTOCOL.md` for the wire contract. The v164 HUD
requires the new Comma client first; an old client rejects the new hello.

## Validation Status

Local Python transport/bridge tests cover bounded pipelining, partial frames,
late/replayed/corrupt ACKs, queue bounds, lease selection, real socket receipt
timestamps, worker cleanup and non-renewal of phone feedback. Legacy USB,
Bluetooth-only and Jetson fallback tests remain in the regression suite.

Local results before device installation:

- 546 Python tests passed; five native-only tests excluded as explained below.
- Actual Python pipeline to production Java parser/owner over a host socketpair:
  15 ordered states at 5 Hz, 15 ACKs, no display clears, clean reader/child exit.
- Java parser: 1,247 assertions; pipeline: eight cases/40,249 assertions;
  display-power policy: 70 fake-actuator checks.
- Deterministic 10 s / 278 ms round-trip simulation: old stop-and-wait 35 clears;
  pipeline with true ACK arrival timestamps zero clears. Using display-poll
  timestamps instead caused 47 clears, so the IO receipt timestamp is essential.
- Independent review found no additional actionable issue after the IO-reader
  correction. Production Python files and new tests pass Ruff; pre-existing
  compact test formatting in the older test files remains outside this patch.
- Signed HUD v164 APK SHA256:
  `9ad7dc0ff175d6de2d0a1fb09d01f06188971d2555a9269d2737b6d97c9af429`.
  Existing signer preserved. Only the Wireless class families in `classes4.dex`
  changed; 7,060 other ZIP payloads and all model-navigation `classes5.dex` bytes
  remain identical. 15,720 non-Wireless classes pass canonical comparison.
  All 162 replacement-class platform references pass API21 checks; tooling's
  22 tests pass. This is not an Android ART or live-radio execution result.

Five native DBC/door tests cannot run in this macOS environment because
`openpilot.common.params_pyx` is unavailable. They are not counted as passes.
No claim of vehicle, optical-display, ambient-light or long-duration Bluetooth
validation follows from these local tests. Parked installation and observation
remain required; no runtime service has been changed by this patch preparation.

## Device Installation and Follow-Up

On October 21, after fresh deviceState and pandaStates confirmed ignition OFF,
offroad, controls disallowed and Panda noOutput, installed source commit
`465b2adf73c184bb0ecc6fbb26b05b9c6b48c419` on WIP-FR. Backed up the original HUD
APK, affected source and Bluetooth configuration before updating. Invalidated
the old finalized-overlay marker without deleting its contents. Existing dirty
generated DBC/build files were preserved. Only navdy_bridge was restarted; no
driving model/control process or full system restart was requested.

The new Comma client first reconnected to the old v1 HUD. Then installed v164
with `adb install -r`, preserving app data, signer and pairing. Read the installed
APK back and verified the exact signed hash recorded above. The existing
Bluetooth-only configuration is unchanged; USB was used for installation and
diagnostics, not vehicle-state transmission.

Actual Navdy Android API22 ART execution passed all 1,247 parser assertions,
eight pipeline cases/40,249 assertions and 70 fake-actuator power checks. The
five previously excluded native DBC cases also passed on Comma by executing the
unchanged test function bodies with their declared arguments, without pytest
decorators. These cases use in-memory CAN frames only, not vehicle writes.

Post-install live observation captured 433 offroad samples over 86.594 seconds:
protocol2, one receiver session, zero disconnected samples, zero receiver clears,
zero USB updates and zero nonzero display statuses. RTT median/p99/max was
116.32/185.87/199.46 ms; maximum consecutive observed ACK receipt gap was 431.06
ms, maximum receiver age 231.94 ms and at most two requests were in flight. The
skip counter stayed at its initial value of one. HUD processes remained stable
at the separately sampled checkpoints. The intended 120-second observation was
interrupted by SSH reset; it is not a completed 120-second pass.

On October 22, reconnected to the same Comma on its home network with a new boot
ID. WIP-FR still held the exact installed source commit and navdy_bridge was
running. Navdy was absent from USB and Bluetooth service discovery timed out,
so post-reboot peer reconnection could not be verified. Neither the network
reset nor later peer absence establishes a Bluetooth regression. Optical HUD,
physical ambient-light, active-driving and long-duration validation remain open.

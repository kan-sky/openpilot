# Navdy Ambient and Display Lifecycle

## Confirmed Gaps

Reference: maintained Wayon/Sunnypilot `b7660d1d403938c464d4f14f842449e3f0e64ebd`,
not the older `codex/my-traverse-source` checkout. Its bridge separates ambient
heartbeat and display power from onroad card/selfdrived telemetry. Its GM
offroad watcher reads real door signals even when card has stopped.

The current Carrot bridge did neither: missing offroad carState selected
`ambientState=disconnected`, no reader followed door edges, and no sleep/wake
operation existed. A fresh parked device snapshot confirmed ignition OFF,
controls disallowed, and Navdy `mWakefulness=Awake`, `mStayOn=true`, display ON.
The existing offroad ambient partial wake lock was present, so killing the HUD
application would remove a required part of the working ambient pipeline.

## Comma Changes

- Retain the fresh, 5 Hz USB/Bluetooth/Jetson full-state transport and existing
  active-display freshness limits. No CAN transmission or vehicle control is
  added by the Navdy bridge.
- Read `pandaStates` alongside `deviceState`. Positive fresh ignition or started
  state means awake. Offroad requires both fresh sources to report no ignition
  and not started. Missing/invalid/stale state is explicitly unknown, not OFF.
- Add `displayPower=onroad|offroad|unknown`. Bootstrap and Bluetooth-expiry clear
  payloads do not contain a known power instruction.
- Continue explicit offroad ambient heartbeat even without carState or
  selfdriveState. Do not invent active telemetry, speed, blinkers or control
  availability from retained door state.
- The read-only door worker is independent of connection discovery. It uses the
  reference GM modern/legacy low-speed door messages on buses 0/1/2/128/130 and
  BCM door messages on 0/128. Only newly parsed door samples update the state;
  quiet buses cannot keep an old open bit ORed into a later close event.
- Fresh live carState seeds the transition; `/dev/shm/params/d/OffroadDoorOpen`
  survives a bridge restart in the current boot, not a reboot. Door-handle/CAN
  burst Comma-screen wake behavior is outside this ambient-only port.
- Restore bounded forwarding of the existing Wayon ambient override files.
  Manual override expiry is honored, durable profiles are retained. This does
  not install a new cloud override producer or create unavailable Wayon ACC
  control modes.
- Bridge status includes explicit power state, observed door state and door
  worker errors in addition to transport/source-freshness diagnostics.

## Validation Status

Initial combined host suite: 312 passed (129 Traverse display tests, 166 Navdy
tests and 17 Jetson management/relay regressions). The speed suite provides
isolated Params I/O for desktop CAN imports. Independently, 67 staged lifecycle
and bridge tests passed on the Comma's native Python/runtime, including real
CAN packer/parser door-open/close fixtures. No CAN fixture was published.

A separate native door worker ran against real read-only subscriptions and a
private test state file: GM detected, no error, thread alive and cleanly joined.
This is initialization/runtime evidence, not a physical door-event test.

The complete staged Traverse/Navdy set subsequently passed 295 tests on Comma
in 12.45 seconds. Only private staged imports were tested; production card and
controlsd were not started, and no control/CAN publisher was created. An initial
run had one missing staged `transport.py` source file in a source-inspection
test; packaging all Navdy modules resolved that fixture omission.

An independent review added 12 regressions for offroad cached-carState handling,
ordered CAN door edges, receive-time freshness and initialization recovery.
Final combined host suite: 324 passed. Staged native Comma suite: 307 passed.

## Navdy Companion

The existing APK already contains the reference ambient behavior: door-open
profile, door-close delay (default 20 seconds), ignition-off courtesy (default
120 seconds), open-door limit (20 minutes), profile/manual overrides and fades.
The missing vehicle heartbeat prevented those branches from running correctly.
The receiver consumes actual `onroad`/`doorOpen`/`gear` fields, not merely the
descriptive `ambientState` string. Preserve the existing three-second ambient
data timeout, existing low-voltage/thermal protections and partial wake lock.

v159 maps only the ambient copy from explicit fresh `displayPower`; unknown or
bootstrap data cannot refresh vehicle-state authority. Original UI telemetry
remains independent. Known offroad for 30 continuous seconds sleeps only the
display; known onroad wakes it. USB priority and Bluetooth lease/expiry remain.

Actual Android 5.1 permission inspection proved the HUD signer lacks the
signature-only `DEVICE_POWER` permission. Adding a manifest request or using
reflection cannot grant it. A separate root-owned 5 KiB JAR exposes only fixed
awake/asleep operations over a local abstract Unix socket, with pinned HUD
signer/UID, root peer verification and original monotonic deadline checks.
No arbitrary commands, network listener or Bluetooth control are provided.

The original `/system/bin/install-recovery.sh` is backed up unchanged. Its
recovery logic is preserved with a small background launcher appended. The
launcher waits for boot completion and has bounded failure retries. Original
file ownership/mode/SELinux label are retained and `/system` is remounted
read-only. No boot image, SELinux policy, pairing or phone permissions change.

APK SHA256: `3dd0dbdeab81dfcc84853a3e2eaa1f24dad7901019ce598591cd2b090da982bc`.
Helper SHA256: `c007adcae852388ce3512f1f1fe8151f4991374a12c70c8e241fa2ad341aa657`.
Existing APK signer is retained. All other non-signature ZIP entries except
`classes2.dex`/`classes4.dex` are identical to v158. Host Java checks: 3,405.
Native policy (70), local protocol (72), Android ambient adaptation, eight
negative IPC cases and unauthorized-UID rejection pass. Initial native probes
exposed pre-connect `setSoTimeout` incompatibility and a probe-only main-Looper
initialization omission; both were corrected before APK installation.

Root and actual HUD-UID client probes independently produced display OFF/ON,
with the same HUD processes, retained offroad ambient wake lock and GATT count.
Update installation returned Success; installed APK bytes match the signed
artifact and the configured wireless peer file is unchanged. The first installed
USB test passed 25-second awake / 34-second asleep / wake-again checks, with
199 fresh snapshots. Real ambient logs show both injected door transitions,
successful BLE writes and a module protocol response for session-off. These
were Navdy-only test inputs, not injected Comma ignition or CAN messages.

The first Bluetooth test rejected an expired snapshot before the 30-second
window completed. It correctly invalidated power authority instead of issuing
a stale sleep request. Do not count that attempt as a successful lifecycle test;
subsequent results and reboot persistence are recorded separately below.

After a Navdy reboot, the root helper started automatically without ADB launch,
the existing ambient GATT connection recovered, and production USB telemetry
caused automatic display sleep. The Comma stayed on its original boot and all
managed PIDs were unchanged during that Navdy-only reboot. `/system` remained RO.

A second USB-attached Bluetooth test also timed out, with accepted RTTs up to
477 ms. Do not claim those tests were stable or that bandwidth was the cause.
After the planned Navdy reboot, a separate isolated test disabled only the
verified Navdy USB device and used Bluetooth with no concurrent ADB polling:
fresh offroad reached 34.86 seconds, actual display OFF was independently read
after USB restoration, then a Bluetooth onroad hint woke it. No exchange error,
same HUD PIDs, retained ambient wake lock while asleep, and GATT count 1.
These conditions changed both boot state and USB activity, so the earlier
latency cause is not identified and long-duration wireless stability is not
established. The transport freshness threshold was not loosened to pass a test.

Physical cluster alignment, real ignition transitions and visible ambient-light
behavior require separate observation. Synthetic display tests and BLE protocol
responses are not physical lamp/color, road or closed-loop vehicle validation.

## Cold Comma Startup Follow-up

The first Comma reboot exposed a separate pre-existing dedicated-ADB startup
failure: discovery allowed only 3 seconds, but native cold startup took
3.0528 seconds. Repeated clients timed out just before startup acknowledgement;
the server then exited with `error writing ACK ... Broken pipe`. Actual managed
traffic recovered over Bluetooth, but never regained USB, so the first managed
handover attempt failed its initial-USB prerequisite.

A parked isolated measurement paused only the Navdy bridge and restarted only
its dedicated ADB server on port 5038. Allowing startup to complete enumerated
the verified Navdy successfully. The bridge was resumed in `finally`.

Give only `devices -l` a bounded 15-second cold-start allowance. Package/service
and forwarding commands remain at 3 seconds; display socket and Bluetooth
freshness limits are unchanged. Reconnection discovery already runs in a
background worker while Bluetooth is active. Tests verify the exact command
scope and timeout propagation without reset or inline retries. Final combined
host suite: 326 passed; staged native Comma suite: 309 passed. Install only the
verified transport file with a backup and extend the existing source manifest.

After a second guarded Comma reboot, SSH and fresh managed services recovered
and USB connected automatically without a manual ADB start. Actual managed
offroad USB-to-Bluetooth handover took 3.248 seconds; the following 60.08 seconds
had 297 distinct frames and zero bad freshness/connection samples out of 590.
Accepted RTT median/p95/max were 51.73/128.82/324.90 ms; maximum receiver age
285.66 ms. This is a bounded offroad observation, not an onroad or long-duration
wireless guarantee. RSS was 56.07 -> 54.97 MiB; bridge CPU averaged 7.12% of one
core during this phase, not total Comma overhead.

Restoring the verified Navdy USB device returned to USB in 4.378 seconds, then
40 distinct frames over 8.04 seconds with zero bad samples. All managed Comma
PIDs and boot ID stayed unchanged throughout handover. Independent Android
inspection confirmed display OFF, the existing ambient partial wake lock,
Bluetooth enabled, one GATT connection and the same HUD/helper PIDs. Only the
Navdy USB node was detached; no Panda, CAN publisher or force-onroad was used.

Final read-only verification matched all eight installed Comma runtime source
hashes, fresh USB status (age 0.114 seconds), no door-worker error, and the
installed APK readback SHA256 above. Only the dedicated port-5038 ADB server was
running. `v159` is the local patch revision, not Android versionCode: the
unchanged manifest retains stock versionCode 3095. A final verification harness
initially conflated those numbers; correct identification uses actual installed
APK bytes rather than changing the manifest merely to satisfy that assertion.
No Git commit/push or real-vehicle motion was performed in this task.

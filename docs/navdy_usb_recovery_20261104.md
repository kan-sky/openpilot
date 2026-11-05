# Navdy wired interruption and bounded recovery

## Incident evidence

The 2026-11-04 incident has two distinct parts:

- Navdy last received fresh Comma data at 11:33:09.815 KST. The host ADB
  USB read failed with `Protocol error` at 11:33:11.636. Navdy's existing
  3001 ms freshness watchdog then cleared stale output at 11:33:13.571.
- The dedicated ADB server on 5038 and an unused default server on 5037
  subsequently competed for the offline device. The bridge searched only 5038,
  ignored offline devices, and could neither reuse the other owner nor repair
  the stuck USB transport. Removing the unused server and restarting 5038 did
  not recover it; a Navdy-device-only USB reset did.

Phone connection-state heartbeats remained CONNECTED around the incident. This
is not an independent HCI continuity measurement. Both observed host and Navdy
USB suspended-time counters were zero; host USB power/control was `on`.
The Navdy kernel log covering the initial failure was lost in an earlier reboot.

`EPROTO` alone cannot distinguish cabling, adapters, peripheral firmware or host
controller faults. See the [Linux USB error documentation](https://docs.kernel.org/driver-api/usb/error-codes.html).
No claim is made that the competing ADB server caused the first USB error, or
that software has eliminated the unresolved physical/driver failure.

## Changes

- Query existing 5038/5037 servers through the ADB smart socket without starting
  another server. Reuse the server that verifies the installed Navdy HUD package.
  Unknown server state never authorizes a competing server or a USB reset.
- For one previously verified, exact Navdy USB device that remains offline,
  attempt a targeted ADB reconnect after 10 seconds. After 30 seconds, permit a
  device-only reset with fresh, recognized runtime evidence, both onroad and
  offroad. Ignition OFF is not required for recovery. No controller,
  Panda, Bluetooth, phone pairing, app, OS or vehicle-control reset is performed.
- Recheck ADB ownership/state and USB identity immediately before recovery. The
  reset child independently checks fresh device/panda messages and a valid
  `IsOnroad` value (0 or 1),
  including after opening the exact character device. Unknown or empty panda
  data does not authorize recovery. Online/unauthorized/ambiguous targets fail
  closed.
- Retain verified identity and budget across bridge restarts in boot-scoped
  state. At most two reset attempts per boot, separated by at least five minutes.
  Failed attempts count. Invalid/symlink budget files fail closed.
- Background USB probes use their own subscriber, a serialized recovery monitor
  and a bounded diagnostic queue. They do not block healthy fallback display
  traffic or consume its main-thread messages. Closing a probe cancels recovery,
  including a launched reset child through an inherited cancellation pipe.
  IPC failure denies recovery rather than interrupting the bridge.
- Save bounded USB/kernel/ADB failure evidence off the display thread, with a
  60-second collection limit and at most four 256 KiB local files. Preserve the
  original direct-connect failure before any Jetson fallback error replaces it.

Wired/Bluetooth selection, phone auto-reconnection, installed Navdy APK,
navigation freshness and vehicle control are unchanged.

## Onroad recovery follow-up

The user explicitly requested recovery without turning the ignition off.
The initial deployment below restricted the additional targeted reconnect and
USB reset to offroad; normal reconnect already operated onroad. Runtime now
uses `RecoveryGuard`, which requires fresh valid device/panda messages and a
recognized, nonempty panda list but permits started/ignition-ON states. The
independent reset child uses the same policy. `ParkedGuard` remains stricter
for installation checks; deploying software is not allowed merely because
runtime display recovery is allowed.

Exact Navdy identity, offline-only owner checks before/after opening the node,
mode/cancellation checks, two-attempt boot budget and five-minute cooldown are
unchanged. No host controller, Panda or vehicle-control process is reset. This
does not fabricate fresh navigation values while the display is disconnected.
Unplugged, unpowered or physically failing hardware cannot be guaranteed to
recover. Budget exhaustion stops device resets, not ordinary reconnect attempts.

Local focused regression: **95 passed**. Added tests cover onroad reconnect and
reset dispatch without ignition changes, default helper guard selection,
onroad/offroad transitions during device open, separate parked installation
policy, stale/future/invalid timestamps and unchanged identity/cancellation
refusals. The ioctl is mocked in these tests, not exercised while driving.
The isolated native full suite passed **1479 tests in 109.03 seconds**. The
actual candidate helper imports correctly, reuses ADB 5038 and refuses a healthy
online USB target. Independent static review found no actionable issue in this
diff.

Runtime commit `31eda60b48546b15e8a6b9e2b3f187d003a7153a` was pushed to personal
`WIP-FR` and selectively installed after another fresh parked check, rollback
archive and candidate/Git/installed hash comparison. Only the Navdy bridge was
restarted. Boot ID, wired selection, Bluetooth configuration, unrelated dirty
files and Navdy main/phone-service PIDs remained unchanged.

The **installed** full suite passed **1479 tests in 104.58 seconds**. Its real
module-mode helper again denied resetting the healthy online device. A separate
65-second simultaneous host/receiver observation recorded **129/129 connected
USB host samples**, 12 fresh USB-owned receiver samples, and **278 received =
278 applied updates**, with zero new clears, rejects, clear failures or actuator
failures. Maximum host status age was 0.2058 seconds; the bridge run stayed the
same. Automatic reset count remained zero, as expected for a healthy connection.
The launch guard still blocks the older finalized update; no staged update or
global updater policy was changed.

Onroad state and the USB ioctl remain simulated in regression tests. No actual
onroad fault was injected, no fresh phone reconnection was induced, and optical
screen output was not inspected. Neither these bounded observations nor the
new recovery policy proves that the original physical USB error cannot recur.

## Verification

This section and the initial installed validation below describe the earlier
offroad-only revision, before the explicitly requested onroad follow-up.

Guarded deployment and bounded installed soak are recorded separately below.
Final local focused suite: 69 passed. Final native full suite: **1453 passed in
104.69 seconds**, isolated from production publication/settings. The initial native suite before the
additional evidence collector passed 1431 tests. Its follow-up caught a collector
constructor assuming a diagnostic journal was present; that optional-journal
dependency was removed before deployment. This failed revision was not installed.

The real device's new read-only guard confirms fresh OFF/one recognized Panda,
reuses port 5038 and rejects a reset of the healthy online device. Native evidence
collection writes 64 bounded, redacted rows (11425 bytes) without disrupting the
existing link. Independent source/mock review confirmed the final cancellation,
IPC-failure and descriptor-cleanup corrections with no remaining blocking findings.
An early standalone-file helper probe lacked the repository import path; the
production helper now uses module invocation, and native candidate entry was
verified separately. These tests are not a reproduction of the original EPROTO.

First-incident evidence, raw device logs and test receipts remain private, outside Git.

## Installed validation

Runtime commit `966653359f947ea699acef6da6d561671870f5c0` was pushed to
`leehyuk1108/carrotpilot:WIP-FR` and fast-forwarded on the parked C4 after a
rollback archive, current-state checks and exact candidate/Git hash comparison.
Only `navdy_bridge` was restarted. The boot ID, Navdy main/phone-service PIDs,
Bluetooth configuration, wired-mode parameter and unrelated generated DBC hashes
remained unchanged. The first fetch failed before installation; a subsequent
fetch and all preflight checks succeeded.

The **installed** full Navdy suite passed **1453 tests in 108.97 seconds** with
isolated Params and no vehicle publication. The actual installed `python -m`
reset helper imported correctly and rejected a healthy online target. Its
diagnostic collector wrote 64 bounded rows (11429 bytes). Recovery state retained
the verified device identity with automatic-reset count zero.

The old finalized update is not installed: the launch script's exact
`.git`-newer-than-`.overlay_init` guard blocks that older overlay. No staging or
update policy was deleted or changed, and no full device reboot was performed.

A 300-second installed observation returned **599/599 connected USB samples**,
with **1497 additional navigation-feedback responses**, maximum status/response
age **0.2283 seconds**, no new bridge failure and the same run/boot throughout.
This overlapped the low-priority installed regression suite. Wired replies do
not carry the Bluetooth receiver diagnostics: the first collector's null USB
counters and `receiverFresh=false` were missing evidence, not a measured receiver
failure. Navdy's logcat ring had already overwritten that observation window;
receiver application therefore needs its separate concurrent capture, not an
inference from host connection status.

That follow-up **65-second concurrent capture** confirmed 129/129 host samples
connected by USB and 12 native Navdy diagnostic samples all fresh/USB-owned.
Between the first/last receiver samples, **277 received updates were all applied**;
there were **zero new clears, rejects, clear failures or display-actuator
failures**. The host stayed on the same run with maximum status age 0.2159 seconds.
Navdy's offroad display remained asleep as intended; counters are not optical
onroad-display validation.

New reset qualification, guard races, IPC failure and budget exhaustion are
synthetic tests. The earlier physical reset restored this device, but the new
automatic reset path has not encountered the original physical failure again.
No live driving, optical screen, or newly induced phone reconnect was tested.

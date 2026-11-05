# Navdy radio recurrence, 2026-10-25

This follow-up separates the recurring Comma-to-Navdy transport stall from an
independently reproducible phone connection ownership defect. Bluetooth data,
phone automatic reconnection and existing stale-data deadlines stay enabled.
No steering, braking or model changes are included.

## Paired automatic journals

Private capture `journal-dropout-20261024/capture-025417` contains both devices'
rotating journals and a hash manifest. Session numbers are scoped to their run;
wall clocks are not treated as directly synchronized.

The reported session 663 closed after 3.004 seconds with `deadline/read_header`.
The last request was processed in 15.606 ms; the subsequent read silence was
1559.041 ms. The Comma recovered an acknowledged session after 5.387 seconds.
This does not support a renderer wait or a permanent Comma producer stop.

Across 4466.874 seconds, 38 exchange failures were retained: 37 deadline
closures and one IOException. Phone state was RECONNECTING for 1744.785 seconds
(39.06% of the observed window); 37 closures fell in that state and one in IDLE
124 ms after it ended. There were no CONNECTED phone samples in this window.
This is association, not proof of the radio mechanism or an explanation of the
user's phone-connected driving incidents. The prior byte-matched paired HCI
evidence is described in `navdy_radio_stall_20261024.md`.

## Fast successful connection rejected

The installed APK's `RemoteDevice.connect()` accepts only a CONNECTING status
after the asynchronous connection call returns. The real socket worker can
already publish CONNECTED before that read. The parent then removes its active
connection/listener and reports failure despite an open connected socket.

An isolated shell-UID ART fixture loaded the exact installed APK and its real
RemoteDevice, SocketConnection and ConnectThread. Only endpoint sockets were
fake; no live Bluetooth operation or production service was invoked. The first
native run reproduced `accepted=false`, `owned=false`, no connected callback
and no socket close on explicit RemoteDevice disconnect. The fixture then
cleaned up its orphaned fake socket. Running HUD/phone PIDs, installed APK and
pairing/configuration hashes were preserved.

That first run stopped at a fast-failure assertion: its expected callback count
was not consistent with listener removal in the original implementation. It
was not a candidate pass. The independent slow-success/rejection test passed.

The candidate changes only `RemoteDevice.connect()Z`, retaining a successfully
CONNECTED worker and allowing its existing callback to report success. Other
DEX files, resources and manifest are unchanged. This specific ownership fix
must not be advertised as the root fix for the separate HCI delivery delays.

## Native comparison

The corrected fixture passed all seven cases on the actual device in isolated
shell-UID ART processes: 165 baseline assertions and 166 candidate assertions.
The real phone service and real Bluetooth endpoints were not invoked.

| Case | Installed baseline | Candidate |
| --- | --- | --- |
| Fast success | false, owner lost, open socket orphaned | true, owner retained, one connected event |
| Slow success and duplicate rejection | passed | identical |
| Gated fast failure | Connection event 1, detached RemoteDevice 0 | identical |
| Slow failure | Connection event 1, RemoteDevice event 1 | identical |
| Endpoint rejects connect | no worker/socket/event | identical |
| Cancel while connecting | closed, no retry, one disconnect | identical |
| Actual DeviceSearch.next with isolated queues | requeued waiting | retained, connected |

The fast-failure count is specific to the fixture gate **before** listener
snapshotting; another production ordering can still deliver one event. It is
not claimed as a universal production count. Slow failure preserves the event
needed by the existing retry mechanism; a physical phone reconnection remains
untested. DeviceSearch's constructor, Handler and event sink were deliberately
bypassed, so this is not a service deadlock or complete service lifecycle test.

The runner verified the unchanged installed APK/PIDs/policy and removed its own
temporary APK, JAR, ART cache and named test process. Native result JSON is
retained privately with the frozen fixture build5. The candidate changes only
the intended method, and signing preserves every non-signature payload byte.

## Bounded role experiment

The first pre-change phase aborted after the existing connection closed; no
role-changing command was issued. The HCI trace shows RFCOMM/L2CAP teardown,
then a local-host Disconnect command about two seconds later. This does not
establish a separate RF link failure: it is compatible with cleanup after the
already observed application transport deadline. The peer reconnected with
the original peripheral role, independently confirmed after 35 seconds. The
temporary passive capture exited and its cleanup reported no errors.

A second full trial also lost its original ACL during A1, before any write.
Neither trial supplies a before/after role comparison. The final diagnostic
3/60/3-second capability probe confirmed one Role Change Success, but lost its
ACL about 3.15 seconds into B. The trace contains a peer L2CAP disconnect request,
later local Disconnect, then a Comma Create Connection and peripheral-role
replacement ACL. The tool could not restore a nonexistent ACL and reported
needs-attention; separate delayed readback confirmed the replacement's original
role. This is a recovered current state, not restoration of the historical ACL
or a successful stability comparison. No permanent radio policy was applied.

All parent-owned capture processes exited. No result from a build, mock
state-machine test or a single short observation establishes phone reconnection
or driving stability.

## Deployment

The first update install failed with `INSTALL_FAILED_DEXOPT`. The device's
dex2oat log explicitly reports `No space left on device` while writing classes2
into the OAT file. The original installed APK remained byte-identical, but
PackageManager had stopped its processes. A fresh offroad guard preceded
restarting that original HUD and phone service; both were confirmed running.
No user data, pairing, old caches or applications were deleted.

The installer was revised to stage only its new, hash-pinned APK in the separate
cache partition, with partition/space checks and a known-APK restart on failure.
This avoids an extra APK copy on the small data partition.

The initial space guard used `df` free blocks and was rejected during review.
The device reported 196.7 MiB free, but DiskStatsService reported only 164020 KiB
available to ordinary processes. The Android 5.1 implementation uses
`StatFs.getAvailableBlocks()` ([AOSP source](https://android.googlesource.com/platform/frameworks/base/+/android-5.1.1_r38/services/core/java/com/android/server/DiskStatsService.java)).
The revised guard uses that available-space report and the measured native
directory/OAT size, with twice the OAT size and 20 MiB reserve on data; cache
requires APK size plus 32 MiB. It does not delete old data or ART caches.
Bond keys/type/PIN/local identity and registered phone entries are compared
without exposing their contents. The complete Bluetooth XML hash is retained
separately because discovery legitimately changes non-bond fields.

The reviewed retry succeeded (`pm install -r`, same signing certificate).
Installed APK SHA256 is
`39693a6e862d602e76064bf4a21829ae8ea7a37359dd4b4b99a3ffb0f74cfa9e`.
The installed bytes, registered phones, bond material, transport policy and
peer settings matched the expected values. In this particular interval the
complete Bluetooth XML also remained identical. Its pre-first-failure copy
was not retained, so this does not attest that earlier interval's bond file.
The staged cache APK was removed after verifying its exact hash. No old caches,
application data or pairing files were removed. The retained rollback APK is
outside Navdy on Comma; no automatic downgrade occurred.

## Post-install result: radio fault remains

The 180-second observation retained 357 samples across 179.524 seconds: 337
fresh connected, nine stale, and eleven fresh disconnected. Three exchange
resets occurred during sampling (new HUD sessions 1, 2 and 3); a fourth reset
arrived just after the window during the final check (session 4). The installer
therefore exited nonzero on its final connected assertion **after successful
installation and policy validation**. This is not a DEXOPT failure or proof of
a new APK regression; it is also not a stability pass.

Private paired capture `journal-dropout-20261024/capture-042239` contains both
journals (all collection return codes zero). Comma failures for sessions 1/2/3
match Navdy closures at sequence 406/21/202. The final installer snapshot supplies
Comma's session-4/sequence-190 failure, also present in the Navdy journal. All
four closures are `deadline/read_header`, about 1.54--1.60 seconds after the last
read, following last-frame processing of roughly 15--31 ms. Session identifiers
are scoped to the new process lifetime; pre-install session 1 is not the same
connection. No fresh HCI capture was made for this post-install interval, so it
does not independently localize those four delays below the application.

The original paired-HCI finding remains: transmitted requests were delayed
before reaching the Navdy app. Its exact controller/RF/host cause remains
unresolved. The corrected fast-success ownership race is a distinct verified
defect, not an explanation or fix for all those delays. Phone automatic
reconnection, Bluetooth-only data, freshness deadlines and driving control were
preserved. Real-phone and driving validation are still outstanding.

The final separate read-only check confirmed HUD/phone processes 5603/5623,
exact candidate bytes, unchanged bond fingerprint and no staged cache APK.
Available space returned to about 160 MiB data / 101 MiB cache. Eleven samples
over 20 seconds were fresh Bluetooth-only data in session 6 (received/applied
82 to 182, USB updates zero), and the authenticated encrypted peer had its
original peripheral role. Session 6 itself shows reconnections continued after
the three-minute window; this brief recovered state is not a stability claim.
No Comma runtime source or control process was changed in this follow-up.

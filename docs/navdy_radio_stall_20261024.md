# Navdy paired radio trace investigation (2026-10-24)

## Evidence and scope

The phone is unavailable. Comma and Navdy remain connected by Bluetooth;
USB is diagnostic-only. Preserve phone auto-reconnect, pairing, expiration
deadlines, model and vehicle-control behavior. Raw traces and APKs are private.

A 180-second paired HCI capture on the deployed `d1b5f803` reproduced three
Navdy `read_header` deadline closures. Of 888 status samples, 854 were connected.
The ACL remained established; RFCOMM/application sessions changed 528 to 531.

The same complete request payloads occur at both HCI boundaries, but reach
Navdy after its approximately 1.5-second socket deadline. Timing bounds derived
from the preceding matching request/ACK, rather than assuming equal device
clocks, place six delayed requests at about 1.59 to 5.20 seconds in transit.
This establishes a delay below the Navdy application, not a Comma application
that stopped producing data. HCI timestamps do not alone distinguish RF,
controller firmware, controller-host transport, or Android HAL scheduling.

All three closures overlap Navdy connection attempts to the absent phone.
Eight HCI Create Connection attempts end in Page Timeout after about 5.12 s.
Some Navdy-to-Comma traffic still arrives promptly during those attempts.
This supports investigation of asymmetric lower-level scheduling, but is not
proof that paging caused the original phone-connected driving incident.
That earlier session 431 had display clears without transport resets and must
be evaluated separately. The prior exact two-null ACK correction is a separate
confirmed input defect; it did not eliminate these offroad transport failures.
Other October 23 onroad records do contain connection resets, followed by the
ambient three-second watchdog and OFF request. They establish the visible
failure sequence, but lack paired HCI/phone-state evidence at the incident.
Do not classify every driving flicker as either this radio delay or a UI-only
discard based solely on the session 431 example.

The offline decoder distinguishes Linux monitor controller indices and retains
only hci0 for this comparison. Counts of Number Of Completed Packets are not
treated as direct air-link latency measurements.

## Confirmed source changes

`d4b1d1b4` records address-free setup phase durations in live status and the
bounded persistent journal. Each connection still discovers the requested UUID
with fresh SDP. A proposed channel cache was rejected before deployment because
a stale channel could belong to the phone acceptor.

Setup has one 15-second budget, including initial cleanup, while retaining the
existing per-phase 3/5/5/2-second maxima. Expiry cannot start another phase.
OS scheduling/process cleanup can exceed the deadline; this is not a hard
real-time bound. A connection exceeding the total budget is closed even if
individual phases succeeded. No stale data is retained to hide a gap.

Parent rerun: 918 tests passed, five existing native DBC cases excluded on Mac
because `params_pyx` is unavailable. Ruff and diff checks passed. Claude reviewed
the final worktree source with no deployment blocker; the parent verified it
matches the named commit and that only Navdy files changed. This setup change
does not correct the demonstrated HCI delivery delay.

## Rejected single-link policy trial

A bounded three-minute trial changed only the Comma-side peer ACL policy from
7 (role switch/hold/sniff allowed) to 1 (role switch allowed), then restored 7.
This is permission for future modes, not proof of exiting an existing sniff
mode. The actual mode was not independently established.

The trial still had a connection reset: 890 status samples, 878 connected.
One short interval versus the earlier three closures does not establish a
reduction in failure rate. It certainly does not establish elimination, so
the setting was not retained as a fix.

The paired trace has eight phone page attempts/timeouts, no ACL disconnect or
Mode Change, and one read-header closure at 1,594 ms. Matching requests 735 and
736 arrive about 2.46-2.76 seconds late using preceding ACK 734's clock bounds.
Thus the application timeout still precedes HCI delivery under this policy.

Readback was 1 at the end and 7 after restoration, with the same ACL handle.
An end readback alone is not proof of an unchanged policy throughout the trial.
The wrapper reported no errors, no forced child kill, and no manual attention
required. Capture cleanup confirmed logging OFF, old snoop bytes restored,
and unchanged app APK, pairing/configuration and Bluetooth-only policy.

The diagnostic tools passed 67 host tests including cleanup failure injection
and wrapper interruption/restore tests. Claude reviewed the final source before
execution. The parent compared deployed script/JAR hashes; seven imported
functions of the older device guard match the reviewed source by AST.

## Remaining limits

No safe root fix for lower-level delivery stalls is yet established. Global
paging timeouts, phone reconnect cadence, roles and stale-data deadlines were
not altered. The phone is unavailable, so real phone reconnection success,
navigation transfer and phone-connected driving behavior cannot be validated.
The setup diagnostics are not a claim that those remaining stalls are fixed.

## Device application

The reviewed setup/diagnostic code `d4b1d1b4` was pushed to `traverse/WIP-FR`
and fast-forwarded on the identified Comma with fresh offroad/ignition-OFF/
inactive guards. The parent checked all five changed tracked-file bytes against
the commit and preserved the existing generated DBC changes/untracked files.
Only `navdy_bridge` restarted (741577 to 834976); `carrot_man` remained 741567.
Pairing/configuration hashes were unchanged. The new bridge reconnected with
all eight setup phases successful and journal queue/drop/storage-error counts
zero. Navdy's APK stayed unchanged. Actual `transport_open` journal records
contain the new timings, not only the live status file.

Final 180-second observation at restored policy 7: 886 samples, 880 connected,
sessions 544 to 545, one connection reset followed by reconnection. The last
sample was fresh/connected with received and applied sequence both 372,
USB updates zero, no incomplete feedback, and journal queue/drop/storage-error
counts zero. The new bridge PID remained unchanged. This explicitly confirms
that the remaining transport interruption is not fixed by the setup patch.

The phone service remained alive with new state records and no diagnostic
drops/I/O errors. No active snoop descriptor or temporary capture process
remained; the trial tmux session exited. Phone connection itself is untested.

## Follow-up: direction and completion accounting

The October 24 late-evening investigation retains the installed upper-overlay
fix `59625f304a` and Navdy APK SHA256
`d1268a7bebbecd34e4ca48f1abac2518138f5048ab14d912c9eda53758c26651`.
The new paired application-journal collection is private
`output/journal-dropout-20261024/capture-232422/`. The exact older HCI traces
remain in `output/radio-stall-20261024/{baseline-r2,policy-r1}/`.

An independent integer-timestamp decoder reassembled both L2CAP directions,
validated RFCOMM FCS and application zlib/CRC/length, and byte-matched eight
delayed requests plus four preceding request/ACK clock-bound pairs. Sequence
numbers alone are insufficient: one sequence is reused after a new handshake.
Twelve offline decoder/regression tests pass. This is recorded-input analysis,
not a new radio test or a claim that both device clocks are synchronized.

| Delayed pair | Arrival relative to original phone page timeout | Transit bounds |
| --- | --- | --- |
| baseline 775 / 776 | +46 / +61 ms | 2.54-2.79 s |
| baseline 213 / 214 | -1.13 / +1.90 s | 2.31-5.20 s |
| baseline 258 / 259 | +43 / +58 ms | 1.59-1.84 s |
| policy 735 / 736 | -2.48 / -2.10 s | 2.46-2.76 s |

The assertion that delivery resumes *only* when phone paging ends is therefore
false. One request arrives during the next page attempt. At least four of
eight baseline pages and six of eight policy-trial pages have no intersecting
request-reception gap of 1.5 seconds or longer. Paging remains correlated with
the observed closures, but paging alone is not sufficient to cause one.

During the gaps, Navdy sends RFCOMM DISC packets that reach Comma in a bounded
0-104 ms. Separate ATT traffic and Navdy controller completion events also
continue. These observations contradict a continuous whole-HCI/HAL freeze;
they do not exclude selective processing, radio retries, controller scheduling
or controller-host flow-control trouble. The delayed requests arrive 375-1391
ms after the first local DISC transmission, not immediately on that host write.

Comma completion events do not identify the individual transmitted request.
The first subsequent same-handle completion report is already 1.59-2.75 seconds
after the corresponding request transmission. Assuming an empty initial credit
queue and FIFO reporting still does not establish reception-before-host-delivery
as the cause. Those assumptions are not demonstrated by a mid-connection trace.
Balanced total TX/completion counts do not establish an empty initial queue.

Neither capture contains Host Number Of Completed Packets, Host Buffer Size or
Set Controller To Host Flow Control commands. Initialization occurred before
capture, so absence cannot establish the configured receive-flow-control mode.
Likewise, a zero Failed Contact Counter is not an RF retry/stall exclusion: that
counter concerns auto-flushed packets, not all retransmissions.

The retained application-log closure while the phone state was CONNECTED
(session 452) aligns with a recorded bridge stop during our earlier deployment.
It is not evidence of a spontaneous phone-connected radio failure. Conversely,
the user's driving symptoms cannot be dismissed: the retained onroad resets
still lack paired HCI evidence. Historical session numbers must be joined with
time/run identity, not counted across process restarts as unique incidents.

No global page timeout, reconnect interval, role, QoS, stale TTL or socket
deadline was changed in this follow-up. A second role trial and a QoS trial
were considered but not run: the existing evidence does not distinguish their
effects sufficiently, and real phone reconnection is currently unavailable.
This follow-up narrows the evidence; it does **not** fix the remaining radio
stall or establish which controller/component is responsible.

## Separate phone connection ownership defect

An isolated shell-UID ART process loaded the exact installed APK, without an
application Context, Bluetooth adapter, actual socket, service lifecycle or
running Looper. It invoked the production `RemoteDevice` and `SocketConnection`
methods with two distinct connection objects and a counting listener. Five
deterministic cases passed their **baseline-reproduction** assertions:

| Injected terminal callback | Replacement still active | Forwarded terminal events |
| --- | --- | --- |
| stale A failure after B installed | yes | 1 failure |
| stale A disconnect after B installed | yes | 1 disconnect |
| current connection failure | no, active cleared | 1 failure |
| current connection disconnect | no, active cleared | 1 disconnect |
| explicit RemoteDevice disconnect | no, active cleared | 1 disconnect |

The first two are defects, not fix passes. The real baseline only guards removal
of the active connection; it still forwards the obsolete connection's terminal
event. Static service inspection shows failure can request RECONNECTING and
queued disconnect handling calls `device.stopLink()` without the original
connection identity. The native probe did **not** execute those service actions,
reproduce a real worker race or establish a connection to the HCI stalls.

A correct fix must also distinguish a legitimate explicitly retiring connection
(the baseline clears active before its callback), and protect the queued-event
consumer if B replaces A after the event is enqueued. Adding only
`source != active` rejection would break normal disconnect/reconnect. Extending
locks into service/link callbacks without checking their lock ordering is not
an acceptable substitute. No such incomplete patch was installed.

The reviewed probe is retained in the local Navdy repository at
`tools/connection-ownership-probe/`. Its test JAR SHA256 is
`a7dcf439cc307b611cce84334b61c072d1824215d370913d6c95c82f9e67d320`;
the runtime classes come from the APK, not replacement stubs. The parent checked
the helper's synchronous dispatch and initialization paths in addition to the
Claude static review. The fixture's synthetic status assignments are not
real phone connection evidence.

HUD PID 2581 and phone-service PID 2601, pairing/configuration hashes and the
Bluetooth-only transport policy remained unchanged. The temporary test JAR,
its own ART cache and any exact-name probe process were cleaned up. The final
status was fresh and connected, session 603, received/applied sequence 1989,
USB updates zero. Earlier real reset/setup-timeout records remain: instantaneous
reconnection is not proof of stability. No runtime code or APK update occurred
in this follow-up, and actual phone reconnection/driving remain unverified.

# Traverse parking impact door exclusion

## Cause and Scope

The initial Carrot port did not enable Wayon's lock exclusion and did not
check physical door switches. The first fix, `28bd2f02`, enabled strict lock
and closed-door requirements but retained a ten-second observation expiry.
That stopped parking monitoring when vehicle CAN became quiet. This revision
supersedes that policy: exclude observed door activity while preserving quiet
parking coverage, without claiming an unknown door state is locked/closed.

Only the parking impact service changes. Driving controls, Panda, model, Navdy,
phone APK, remote-start exclusion, impact thresholds and uploader are unchanged.
Other vehicle fingerprints retain their prior defaults.

## Behavior

- `CHEVROLET_TRAVERSE` defaults to `impact_door_activity_guard=true`. This
  supersedes the two old strict requirement flags. Other fingerprints keep
  their previous defaults. Explicit activity guard false supports the legacy
  `impact_require_door_locked` / `impact_require_doors_closed` policy.
- Receive bus 0, `0x130`, byte 0 mask `0x06` uses Wayon's existing decoder:
  masked value 2 means unlocked. This patch does not invent another encoding.
- Receive bus 0, `0x12A` (`BCMDoorBeltStatus`) checks all four door switches:
  byte 1 mask `0x07`, byte 2 mask `0x80`. Transmit/forward echoes do not qualify.
- Observed unlocked/open states block detection and discard pending candidates
  until the corresponding lock/closed observation. These observations do not
  expire just because the parked vehicle stops sending frames.
- Unknown startup state permits monitoring only after healthy fresh CAN
  receiver, Panda, offroad and IMU checks. Diagnostic values remain null;
  saved state files are never restored as current physical evidence.
- Fresh **empty CAN envelopes** establish receiver liveness, not door state.
  No new receiver envelopes, backlog/invalid messages, Panda restart, harness
  change, new RX loss/SPI errors or current CAN faults invalidate retained
  evidence and cancel candidates. Recovery then needs new door/lock samples.
  Initial subscription waiting is not treated as a lost established session.
- After observed doors close and observed locks engage, retain the 3-second settling delay and the
  detector's 5-second arming/warmup. Open/close in one CAN drain also restarts
  warmup, including when there is no new accelerometer sample.
- Hold guarded impact candidates for 0.5 seconds before queueing. A door edge,
  missing guard, IMU interruption, ignition or incomplete CAN drain cancels
  them. Preserve the original event timestamp for remote-start exclusion.
- Recheck offroad, receiver health and door activity immediately before queueing an impact.
  Lock/unlock notifications are not blocked by the impact-only queue check.
- Process restart never trusts saved lock/door files as current evidence.

The legacy ten-second expiry applies only when activity guard is disabled.
The activity policy intentionally trades proof-of-locked for continuous quiet
parking monitoring. Unobserved doors at startup, a physically broken CAN wire
without reported hardware errors, or a lost/delayed door edge beyond the
0.5-second confirmation window can still cause a door-related false alert.
Receiver heartbeat is not proof that every physical vehicle signal is present.

## Verification

The previous strict revision passed 363 isolated native tests; that did not
prove quiet parking coverage. This revision passes 390 tests on the Comma's
native Python/Cap'n Proto runtime. Expanded tests cover quiet startup and
30 seconds of continuous IMU with empty CAN, six-hour observation retention,
door edges during impact confirmation, settling/rearm, malformed frames,
receiver interruption, Panda faults/restarts and final queue guards. Tests do
not write production queues, send notifications or actuate the vehicle.

Recorded route `000001aa--5e1e07e506`, segments 0/16/17:

| Signal | Received | Result |
| --- | ---: | --- |
| Bus 0 `0x130` lock frame | 166 | All masked value 2, blocked as unlocked |
| Bus 0 `0x12A` door frame | 1,661 | Every result agrees with native GM DBC parser |

Recorded door transitions also align with `carState.doorOpen` transitions.
The sampled logs contain no physical lock-to-unlock transition, so they prove
frame reception and parser agreement, not that both physical lock states have
been verified. Locked/closed rearming and impact release are isolated tests.

The earlier 15-second observation received 1,502 CAN envelopes and zero frames.
In this revision's live read-only shadow check, 65.007 seconds of real IMU/CAN/
Panda input reached armed after 5.054 seconds and stayed armed for the remaining
5,979 monitor updates. Zero door frames and zero events were observed. Unknown
door/lock fields remained null. This is bounded native monitoring evidence,
not a physical door/impact trial or a guarantee of overnight continuity.

Runtime diagnostics are `/data/wayon_cloud/impact_status.json` and
`door_lock_status.json`. The latter now includes `doorOpen`; the former also
reports `doorActivityGuard`, `doorStateObserved`, `doorStateRefreshRequired`
and observation ages. States distinguish receiver waiting, observed open or
unlocked, settling, warming and armed. Config backups must stay private because
they contain cloud credentials.

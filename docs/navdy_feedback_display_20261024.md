# Navdy feedback and partial-write corrections (2026-10-24)

## Scope and findings

Published runtime: `9dc436ca811d2896ef13f1aaa0abad7ba7a7c7da`, `WIP-FR`.
Baseline: `bf76c8b96971309ee29ffa107b4e72601937eb29`.

The exact response `{"navigationFeedback":null,"modelNavigation":null}` is
Navdy's discarded-display-frame acknowledgement, not a new navigation clear.
Previously the bridge saved this as the latest observation. A normal camera,
discard, normal-camera sequence reproducibly became present/absent/present.
This is a demonstrated input defect, not proof that it caused every recorded
24-second navigation gap or every missing camera.

The bridge and readers now ignore only that exact two-key response. They select
the last genuine observation in a batch and preserve its original receipt and
producer deadlines. Real camera/guidance/model clears and malformed responses
still use the existing clearing behavior. Invalid model aging cannot convert a
real guidance clear into the special discard shape. This changes navigation
input availability within its original validity window; it does not change the
driving model, planner, steering, braking, DM or Panda algorithms.

Successful socket short writes now continue for at most 16 attempts per flush.
EAGAIN returns immediately, and every attempt checks the original 500 ms TTL.
Frame frequency, maximum in-flight requests, 1.5-second response timeout,
producer/link deadlines and Bluetooth/phone reconnection policies are unchanged.
No additional thread or unbounded send queue was added.

The historical HCI/read-header interruption is not explained by this change:
in the earlier capture complete requests had already reached the Comma HCI
interface. Actual short writes during the reported drive remain unproven.

## Verification

- Final host suite: 838 passed, five DBC cases excluded because the Mac lacks
  `params_pyx`. The same five unchanged cases passed with the real Comma native
  DBC packers/parsers, without CAN publication or vehicle commands.
- Real socket framing, fragmentation, plain/zlib decoding, receipt timing,
  status 0/1/3, batch ordering, real clears, malformed timing and original
  deadlines were exercised through the actual pipeline, bridge and file reader.
- Actual Python pipeline to compiled v164 and v167 production Java receiver
  fixtures passed separately: 15 consecutive applied frames each, no clears.
- Ruff and whitespace checks passed. Claude independently reviewed the fixed
  commit and raised no remaining deployment blocker; this was static review,
  not an independent device or road test.
- Device fast-forward preserved generated DBC changes and untracked build
  artifacts. All ten changed tracked-file hashes matched the reviewed checkout.
- Fresh valid/alive/fresh Park, zero speed, parking brake, standstill and inactive
  state were checked before update and before service restart. Only
  `navdy_bridge` and `carrot_man` restarted; the manager spawns fresh interpreters.
  No full reboot, model restart, forced engagement or synthetic CAN was used.

## Bounded device observation

Both intervals were onroad but parked and disengaged, with the real phone
connected. USB was diagnostic-only; display traffic remained Bluetooth-only.

| Measurement | Before | After |
| --- | --- | --- |
| Interval | 179.899 s | 179.903 s |
| Connected status samples | 887/887 | 891/891 |
| Receiver session | 452 throughout | 453 throughout |
| Receiver clears | 1198 to 1198 | 1199 to 1199 |
| RTT p50 / p95 / p99 | 130.00 / 184.30 / 196.52 ms | 122.37 / 168.29 / 184.93 ms |
| Receiver-age p50 / p95 / p99 | 107.13 / 164.48 / 193.84 ms | 100.61 / 152.07 / 170.14 ms |

The one session/clear increment between intervals is the intentional bridge
restart. The baseline's last-failure object was historical, not 887 failures.
Neither interval reproduced the reported dropout, so the latency difference is
not evidence of causal improvement. Afterward partial-write, blocked-write and
no-observation counters were all zero; journal drops and storage errors were zero.
The actual phone service remained CONNECTED throughout the captured heartbeat
window. This does not prove phone reconnect behavior or long-drive stability.

## Phone camera boundary

At the observed stationary location the phone selected Camera Clear, Navdy also
reported Camera Clear, and Comma reported camera speed zero with a separate
type-22 speed-bump observation. Those values agree; no real enforcement-camera
loss was reproduced. Native UI secondary-slot behavior differs from the custom
selector in some inputs, but the freshness of that secondary slot is not yet
established. Type zero must not simply be reclassified as a camera, nor should
the old minimum-distance selection of retained slots be restored.

A separate diagnostic-only patch for the original `com.skt.tmap.ku` records
bounded selector-time slot fields and rejection/selection reason. It preserves
the existing selector and stale tracker. Host 12 tests/18 diagnostic checks and
actual phone ART before/after 24-case replay passed with identical results.
Only one canonical class in `classes21.dex` changed; 131 other classes in that
DEX and 9,636 other APK payload entries were preserved. The original certificate
and application data were retained. The separate `com.skt.tmap.cn` was untouched.

The diagnostic APK installed successfully and its device hash matched
`f18007ca67143749d5c310ba7e2083164d26499eee142d13234934cbd7aa0cef`.
The separate app's installed path and hash were unchanged. Actual ART emitted
a diagnostic with real RGData/SDIInfo field names. This was isolated replay,
not a real camera encounter. Immediately after relaunching the installed app,
the phone disappeared from USB and the Mac changed LAN subnets. Live app log
capture therefore did not complete. Guidance was absent in the last Comma
snapshot after app installation; route resumption must be verified, not assumed.

Phone diagnostic sampling is at most once per five seconds; shorter events can
be missed. Logs include numeric camera identifiers and driving metadata, so
they are private even without explicit coordinates. APKs, credentials, raw
route data and device logs are not committed here.

## Remaining acceptance criteria

### Later observation supersedes any whole-session stability claim

At 19:17 KST, after the user's phone had left and the Mac had changed networks,
SSH through the verified Tailscale address still reached the same Comma and
runtime commit. The saved 16:54:19-19:17:56 interval contains 55 exchange
failures plus 11 failed connection attempts. All 66 failure records report
offroad. Matching Navdy sessions show 55 `read_header` deadline closures.
The nearest preceding phone-state sample (at most ten seconds old) was
RECONNECTING for 54 and LISTENING for one; this is correlation, not proof of
the controller/air-link/receiver root cause or an explanation of the earlier
phone-connected driving incident.

All captured failure snapshots have zero short-write and blocked-write counts.
Thus the bounded-send correction did not resolve this later interruption path.
The new live counter recorded 72 no-observation responses with zero incomplete
responses. Journal drops/storage errors remained zero. The early three-minute
pass must not be presented as proof that all flicker has been fixed.

The user confirmed that phone/Navdy reconnection is not currently possible.
No further radio-policy, phone-search or receiver-timeout experiment was
deployed without that end-to-end validation. Bluetooth-only transport, phone
automatic reconnection and existing expiration deadlines remain enabled.

The reported engaged-drive display/ambient flicker and intermittent missing
camera still require an actual occurrence with matching phone, Navdy and Comma
logs. New counters separate no-observation ACKs, genuine clears, partial writes,
socket backpressure and ACK-poll delay without extending stale-data validity.
Do not interpret this parked test or the phone replay as physical display,
camera braking, or complete road-reliability validation.

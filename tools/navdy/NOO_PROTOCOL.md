# Navdy Model Navigation (Development)

This is separate from the legacy display/navigation feedback. Legacy turn
numbers or receipt times MUST NOT be promoted into model provenance. The
existing phone RFCOMM owner and comma USB-first/Bluetooth transport stay intact.

## Historical Phone To HUD v1

A dedicated `WAYON_MODEL_NAVIGATION_V1` notification carries a JSON object:

- `version`: integer 1; `source`: `tmap` or `naver`.
- `session`: producer-process UUID; `sequence`: increasing native guidance
  observation number, at most 2^53-1. Re-sending bytes never increments it.
- `observedAtMs`: original observation wall time. The phone retains monotonic
  age locally and rejects expired queued observations before writing.
- `active`: boolean; `offRoute`: boolean. Inactive, off-route or unknown route
  immediately clears model guidance. Display guidance remains independent.
- `routeId`: a new identifier (at most 64 characters) for each route generation,
  even when geometry is unchanged. No guessed chord or truncated route is accepted.
- `maneuver`: canonical `type`, `modifier`, `distanceMeters`. Unknown native
  maneuver codes disable model guidance; display mapping remains unchanged.
- Optional `position`: latitude, longitude, `bearingDegrees`, `speedMps`, and
  ORIGINAL `observedAtMs`. Missing direction is not synthesized as north.
- Optional `route`: `polyline6`, `pointCount`, `sha256`. The SHA-256 covers the
  complete ASCII polyline6 (latitude then longitude). Sent on route change/reconnection,
  plus bounded recovery re-offers on fresh native observations, not on every
  display update. A successful socket write does not prove receiver acceptance.
  Maximum 8192 points and 65536 encoded bytes.

The HUD validates native observation identity, sizes, numbers and source before
storing. At first receipt it bounds age using original phone wall time; accepted
clock uncertainty must be charged conservatively, not extend freshness. After
receipt all expiry uses monotonic time. Duplicates cannot extend deadlines.
Unsupported clock/skew or absent original timestamps fail closed.

Live testing found that the devices' wall clocks can differ by more than the
v1 future bound. Sender-only time synchronization also does not bound receiver
queue delay, and a delayed backward clock step can understate an old packet's
age. V1 is not the release freshness contract for the replacement channel.

## Phone To HUD v2

The v2 channel keeps native producer identity, sequence, original timestamps,
route integrity and size limits, but does not rely on phone/HUD wall-clock
agreement for model freshness. It uses three reserved notification sources:

- `WAYON_MODEL_NAVIGATION_CHALLENGE_V2`: HUD-originated unpredictable nonce.
- `WAYON_MODEL_NAVIGATION_RECEIPT_V2`: phone receipt for bounded transport
  bookkeeping only; it never creates or renews navigation guidance.
- `WAYON_MODEL_NAVIGATION_V2`: native observation plus its matching nonce.

The HUD has at most eight outstanding transport credits. Challenge validity is
at most two seconds, but expiry alone does not free an unreceipted queue slot.
Receipt or proof releases its credit; reconnect clears the old connection.
The nonce itself is retained after receipt until consumed, invalidated or its
original two-second deadline expires. Receipt alone must not evict a still-valid
proof. Retained nonces share the same eight slots, so issuance may wait for a
slot; no separate unbounded pending queue is introduced.
The phone claims a credit before dispatch, so a reverse response arriving before
write completion cannot recreate a stale pending receipt. The phone
may bind an observation only to a challenge received strictly before the
observation's original monotonic capture time, in the same connection epoch.
It must not reply with cached guidance captured before that challenge, fabricate
a new sequence/time, or use receipt/admission time as native capture time.

The HUD charges the entire time since challenge issuance as observation age.
This is a conservative upper bound, including outbound transport, waiting for
the native observation, inbound transport and receiver scheduling. Expiration
is anchored to challenge issuance plus two seconds, not receipt. A nonce is
single-use for active guidance, is connection-scoped and expires even if its
transport receipt never arrives. Disconnect invalidates outstanding challenges.
No challenge or receipt alone enables model input. V1 is not an automatic
fallback when v2 proof is absent. Ordinary display TBT remains independent.

The current phone producer omits GPS because its original fix freshness is not
proven. V2 must not use a guidance nonce to make an older position fix fresh.
The downstream HUD-to-comma envelope remains version 1 with conservative
`ageMs` / `validForMs`; it does not require another comma schema change.
Protocol support does not imply inference or driving validation;
see the dated verification record for exact installed hashes and test results.

### Verified Geometry Receipt

A challenge may include `routeCache`, either null or an exact six-field object:
`source`, `session`, `routeId`, `sha256`, `pointCount`, `encodedLength`. Only
complete, verified current HUD geometry can produce it. The entire control must
remain plain ASCII and no more than 512 bytes; oversized escaped identities
fall back to null. The new phone still accepts the original three-field control.
Install the new phone parser before enabling this extension in the HUD.

Matching metadata grants the phone a transport-cache lease of at most two
seconds from receipt in the same connection epoch. This suppresses unnecessary
periodic full-route reoffers. It does not create a native observation, renew
guidance, or satisfy the challenge proof. Null, mismatched or expired cache
evidence requests bounded recovery on a genuinely newer native observation;
the original backoff remains, rather than repeatedly flooding the route.

## HUD To Comma

The normal ACK gains `modelNavigation` (object or null). Existing fields retain
their format. The object carries `version`, `source`, `session`, `sequence`,
`active`, `offRoute`, `routeId`, `ageMs`, `validForMs`, `maneuver`, optional
`position` (with independent `ageMs`), and optional `routeChunk`:

```
{"encoding":"polyline6","index":0,"count":4,"pointCount":300,
 "encodedLength":3400,"sha256":"...","data":"..."}
```

Each chunk is at most 1024 ASCII characters; all except the last have exactly
1024. There are at most 64 chunks. The comma request's optional `modelRoute`
object contains `routeId` and `index`; a missing/mismatched ID requests index 0.
Once complete, index -1 suppresses route transfer. Chunk transfer never delays
or replaces ordinary display/feedback frames. Total ACK limit stays 32768 bytes.

The host reassembles one route only, verifies exact length, SHA-256, point count,
coordinate bounds and complete polyline termination before publication. No
partial path is usable. Source/session/route changes clear old geometry. A
missing component, inactive/off-route state, stale source or disconnected link
clears model readiness immediately. Reconnection requires geometry again.

Both USB and Bluetooth charge the full measured exchange duration to model
feedback age and subtract it from remaining validity. Position keeps its own
original age. Polls never renew an unchanged source sequence. Guidance is
limited to 2 seconds from source observation and link silence to 2 seconds.

## Comma Local Cache

The bridge writes a complete sidecar with a random `bridgeCacheToken`, then
publishes the matching `modelRouteCacheToken` in the outer local feedback file.
Every bridge clear/reassembly rotates this token, even for unchanged route
identity/hash. Readers require matching tokens and fresh native observations
before importing geometry; reader-only expiry does not permanently blacklist
an otherwise unchanged bridge assembly. These tokens never travel to the HUD
and are consistency markers, not authentication credentials.

For a missing or expired guidance component only, the tracker can quarantine
one previously fully verified immutable route for ten seconds from the first
expiry. It is neither ready nor publishable while quarantined. Restoration
requires a genuinely newer fresh native sequence in the same source/session/
route, plus a valid chunk whose full manifest and exact bytes match that route.
No chunk, a replay, or a hash-only claim cannot restore it. Repeated missing
components never renew retention. Link silence/disconnect, explicit clear,
invalid data, cancellation, off-route and identity changes discard quarantine.
Restoration rotates the local sidecar token; old reader tokens remain invalid.

A repeated same-high-water ACK can cross expiry when its complete round-trip
time is charged, even before the tracker's previous deadline. A structurally
valid tail can quarantine already verified geometry or preserve its quarantine,
but cannot restore readiness. Retention starts no later than the earliest of
the previous deadline, that tail's deadline and the current time; subsequent
tails cannot extend it. Invalid fields, conflicting chunks, changed identity
and a newer but already stale sequence still discard the candidate.

## Validation Boundary

Unit tests and offroad transport tests do not establish N16 inference or driving
performance. Jetson inference must be checked separately with the actual device.
No forced-onroad or vehicle actuation is part of this protocol's validation.

# Navdy Wireless Link

Implementation contract, 2026-10-19. This is distinct from the synthetic bench.
Secure bonded RFCOMM UUID: `9b07004c-c95d-4e82-bf6f-9af7679b4a21`.
Both ends allow exactly one explicitly configured bonded peer; never auto-pair,
scan arbitrary peers, reset an adapter, or modify phone/ambient services.

All integers are big endian. Payload bound is 32768 bytes before AND after
compression. UTF-8 is strict. JSON must be an object; request schema is
`navdy.openpilot.v1`. Only full state snapshots, never arbitrary commands.
Compression flags: 0 raw, 1 zlib level 1. CRC32 covers uncompressed bytes.
Reject truncated, oversized, trailing or over-expanding compressed streams.

- Server hello `!4sIII`: `NVH1`, lease=0, TTL=500000 us, bound=32768.
- Request `!4sIIqIIII`: `NVQ1`, increasing sequence (0..2147483647), current
  lease, signed captureDeltaUs, raw length, wire length, CRC32, flags, then wire.
- Response `!4sIIIqIIII`: `NVA1`, matching sequence, status, next lease,
  receiver age upper bound in us, raw length, wire length, CRC32, flags, then wire.
- Status 0 means actually dispatched on the main thread while fresh; 1 expired;
  2 invalid ordering; 3 USB currently owns the display. Reject invalid frames by
  closing the session. The response JSON is the EXISTING camera/roadPreview/
  navigationFeedback ACK, optionally with a `wireless` diagnostics object.

Server records its monotonic lease issue time immediately before hello/ACK.
Client records monotonic receipt after reading the complete response. Client
captures a new full payload after connecting and sends captureDeltaUs relative
to that receipt. Server age upper bound is elapsedUs - captureDeltaUs + 2000 +
max(0, elapsedUs/1000). Reject abs(delta)>10000000, negative age or age>=TTL.
Increment lease after every well-framed response; new socket resets the session.
Recheck remaining lifetime at main-thread dispatch, not just at socket receipt.

USB takes priority. Existing USB main-thread apply records USB ownership. While
USB was updated in the preceding 750 ms, Bluetooth must not alter the HUD.
Bluetooth retains at most one pending main-thread update and one in-flight
transaction. An independent 50-100 ms expiry check clears Bluetooth-owned state
at its remaining TTL or on disconnect, never newer USB state. New USB updates
invalidate queued Bluetooth work; a closed/replaced session cannot update or
clear a newer owner's display. ACK must not claim success before actual dispatch.
Preserve existing phone-navigation source session/sequence/age; no freshness reset
of cached feedback. Clear uses existing inactive/offroad payload behavior.

## Bounded Pipeline v2 (2026-10-21)

The v164 HUD advertises `NVH2`; request/response markers become `NVQ2` and
`NVA2`. Struct layouts, encryption, exact peer, CRC/decompression bounds and
the 500,000 us display lifetime are unchanged. The Comma client still supports
an old `NVH1` HUD through the original synchronous protocol. An old client must
not interpret `NVH2` as v1; update the client before installing the new HUD APK.

V2 allows at most **two** unacknowledged full snapshots at the existing 200 ms
capture cadence. It does not wait for an ACK before capturing the next state.
When both slots are occupied, discard that tick, not an unbounded FIFO of old
states. Only one partial transmit buffer is retained; its original capture
deadline remains 500 ms. No catch-up burst or cached active-state retransmission
is allowed. The server still has only one pending main-thread dispatch.
One nonblocking client IO reader timestamps the complete ACK as it arrives and
retains at most two responses. It never applies app state. The 200 ms display
tick must not be used as the ACK arrival time: that polling delay would inflate
the next lease's conservative age and reintroduce expiry. The reader has bounded
shutdown and is closed with its transport before any handover/reconnect.

Each ACK issues lease `request.sequence + 1`. The server retains the last eight
immutable lease issue times, allowing two requests to reference the same lease.
Capture delta refers to the most recent lease the client knew **at capture**,
not an ACK received later while polling. Strict sequence ordering, receipt-age
and main-dispatch checks, USB ownership and expiry checks remain mandatory.
An accepted display does not get a new lifetime when its ACK is written.

The client waits at most 1.5 seconds from original capture for a complete ACK.
This is a transport deadline, **not** an extension of display validity. Status
1 (expired display) and 3 (USB ownership) still issue a lease and independently
timed phone feedback, so they do not tear down a healthy socket. Invalid
ordering, framing, CRC, age, oversized data, EOF or an absolute timeout close
the session as before. Polling cannot renew the deadline of partial responses.

Each real response is consumed once. Charge capture-to-complete-receipt delay
against navigation and model-navigation freshness; an empty poll never writes
a new feedback receipt timestamp. Existing phone sequence/session, route cache,
model-navigation validity and route-completeness rules are unchanged.

This addresses observed stop-and-wait expiry and local timeout-triggered
reconnects, not every possible Bluetooth or power fault. Ambient's separate
three-second missing-vehicle-data shutdown remains unchanged. Do not suppress
that shutdown or preserve stale engagement indicators to disguise packet loss.

Comma uses direct USB first, then the configured Bluetooth peer, with existing
paired Jetson USB relays retained as further fallbacks. Reset fallback ordering
after a successful USB exchange so each new cable loss tries Bluetooth first.
While on Bluetooth, probe USB in a background
worker and hand back only once a real connection is ready. Never send stale
payloads captured before discovery or retain queued state across a reconnect.
Never run two live display writers. Failed wireless sessions clear local
navigation feedback, close their socket and retry with bounded backoff.

Navdy loads the allowed comma MAC from its private files directory's
`openpilot-bluetooth-peer` file. No file means wireless disabled. Its listener
starts with the existing OpenpilotStateService and retains existing USB support.
Keep the enabled adapter connectable (not discoverable) for the paired peer;
do not turn on an explicitly disabled adapter or remove any bonds.
Comma config is `/data/navdy/bluetooth.json`: `{"enabled":true,"peer":"AA:BB:CC:DD:EE:FF"}`.
Without that config, existing USB/Jetson behavior is unchanged.

## Display Power and Ambient Lifecycle

The optional `displayPower` field is `onroad`, `offroad`, or `unknown`. It is
independent of active control and UI telemetry: fresh ignition or started state
keeps the display awake; known offroad requires both fresh device and panda
state to be false. Missing, invalid, expired, bootstrap or disconnect-clear
payloads are unknown, not a sleep command. A retained offroad door value can
update ambient lighting but cannot grant active telemetry or wake authority.

The v159 HUD receiver requires 30 seconds of continuously fresh known offroad
state before display-only sleep. Fresh known onroad requests wake immediately.
USB and Bluetooth share the same owner and policy. Receipt/main-queue delay and
the existing 500 ms lease count against power freshness; do not renew an expired
instruction while handing it to a native helper. Keep the HUD process, ambient
partial wake lock and existing phone/ambient Bluetooth services running.

On the verified Android 5.1 Navdy, the HUD signer does not hold the platform's
signature-only `DEVICE_POWER` permission. The companion root-owned standalone
helper uses an abstract local socket, `hl-navdy-power-v1`, not a network port.
It checks the installed HUD UID and signer, accepts only that UID, and exposes
only fixed awake/asleep requests with sequence and monotonic deadline checks.
The client verifies root peer credentials. No shell, file path, radio reset or
arbitrary command is accepted. Helper responses distinguish observed state,
already-correct state, rejected/expired requests and unconfirmed operations.

The helper is a separate device prerequisite; installing only Comma source or
only the APK does not grant that permission. Preserve its root-owned JAR and
the original device recovery startup script when provisioning or rolling back.

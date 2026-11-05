# Automatic shutdown phone notice

The manager sends a best-effort My Traverse notification immediately before an
already-authorized `low_voltage` or `offroad_timeout` shutdown. Existing 11.3 V,
10-second qualification, offroad duration and final ignition/freshness guards
are unchanged. No manual poweroff, driving control or Navdy path is changed.

- The isolated sender has a one-second wall-clock budget, including imports,
  DNS and TLS. It is killed and reaped on timeout. Notification failure does
  not block battery protection.
- Use the existing enrolled Wayon identity, without enrollment or credential
  changes. The lightweight read-only DongleId fallback avoids native Params
  startup; stdlib HTTPS avoids expensive requests imports. Redirects are refused.
- Recheck fresh device/Panda/voltage state after delivery. Ignition, recovered
  voltage, withdrawal and expired requests cancel automatic shutdown.
- Cloudflare authenticates the device, records an idempotent event in the existing
  D1 table and returns 202. `waitUntil` finishes FCM independently of the Comma
  connection. Acceptance is not a phone delivery receipt.
- My Traverse Next v37 adds a high-importance power notification channel and
  handles foreground delivery. Background FCM carries the same visible text.
- Text says shutdown is pending, not completed. Offline networks, denied phone
  permissions or abrupt hardware power loss can prevent a timely notification.
  There is no promise that the phone displays it before physical poweroff.

Cloud source is a hash-pinned addition to the actual production Worker, under
`cloudflare/wayon-cloud/patches/shutdown-notice-20261030` in the My Traverse
source repository. Do not deploy its unrelated dirty `src/worker.js`.

## Validation

Initial native tests exposed >1 s native Params/requests startup, causing both
real send attempts to time out before server acceptance. Lightweight identity
access and stdlib HTTPS correct that path without increasing the shutdown wait.
Final real Comma test requests for both reasons were accepted in 0.873/0.872 s;
both corresponding notification IDs and reason text appeared in Android's
notification records on the intended phone. Every test explicitly states that
the device will not shut down. No shutdown condition was injected into live Params.

Local and isolated Comma regression suites cover the actual manager function,
both reasons, failed delivery, ignition/voltage recovery and stale requests.
Worker tests cover authentication, concurrent deduplication, async FCM failure,
and preservation of existing lock-event handling. Workerd/D1 isolated checks
exercise the actual Worker request router. APK release tests/build pass, and
v37 was installed with the existing signer using `install -r`, preserving data.

Real low-voltage/72-hour poweroff is deliberately not forced. Tests replace the
shutdown callback; software activation and actual poweroff are separate claims.

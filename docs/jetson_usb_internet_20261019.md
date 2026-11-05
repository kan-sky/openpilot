# Paired Jetson Internet over the existing USB cable

## Observed failure

On the tested WIP-FR installation, the comma and NX 8GB could exchange USB
boot-gate messages, but the Jetson had neither a Wi-Fi interface nor a default
route. DNS resolution of the signed update host failed. The existing mobile
NCM helper intentionally supplies DHCP without a router or DNS: a USB cable
alone did not share the comma's Internet connection.

The replacement image also changed the Jetson SSH host key. The comma retained
the previous pinned identity and address, so its Navdy relay could not connect.
The old relay path was absent from the protected image. The separately prepared
Navdy management package is available on DATA, outside the signed inference
runtime. Navdy itself was absent from the Jetson USB and ADB inventories; this
is separate from Internet routing and cannot be called an APK/relay failure.

## Scope

- Keep the current auto-mode NCM network, DHCP addresses and bulk descriptors.
- Share IPv4 Internet only with a Jetson authenticated by the existing paired
  SSH key and independently verified host-key pin. Never disable host checking.
- Keep private keys on their original devices. Enrollment adds only a public
  key and preserves existing authorized users.
- Use the comma's current physical uplink and DNS; reject a VPN as the uplink.
- Add only owned firewall chains. NAT is restricted to the authenticated peer.
  Other LAN/VPN destinations are denied, except the configured DNS servers on
  DNS ports. No inbound/public listener or inbound port forwarding is added.
- Enable forwarding only on the two participating interfaces, restoring their
  original values on cleanup. A journal recovers state after a service crash.
- Preserve DHCP on Jetson. Add a default route rather than assigning a static
  address or setting NetworkManager's incompatible DHCP `ipv4.gateway` field.
- Persist its NetworkManager profile through the existing protected-storage
  identity mechanism, without remounting the system writable.
- Correct large clock drift from authenticated comma time before HTTPS; leave
  NTP enabled. The tested Jetson subsequently reported NTP synchronized.
- Do not change model selection, signed-release verification, controls, or DM.

This helper currently targets the existing auto/mobile NCM network. It does not
reconfigure a live USB gadget or enable sharing on the separate legacy
link-local management network.

## Files and lifecycle

`tools/jetlink/usb_internet.py` owns the routing service and bounded peer setup.
`tools/jetlink/hl-jetson-internet.service` is copied into `/run/systemd/system`;
the comma OS stays read-only. The existing `jetson_maintenance` process starts
it when `/data/hl-jetson/usb-internet.enabled` is present. Thus the source-backed
startup hook recreates the volatile unit on the next normal manager/OS start.
Do not claim an actual full reboot test from this wiring alone.

Install the same helper, root-owned, at
`/run/carrot-data/hl-network/usb_internet.py` on the protected Jetson. Its profile
comparison cache avoids repeatedly rewriting persistent configuration.

The optional paired configuration `navdy_relay: protected-tools-v1` selects the
fixed, integrity-checked `/run/carrot-data/hl-navdy/navdy_management.py relay`
endpoint. Legacy installations retain their existing signed-runtime endpoint.
USB candidates are refreshed, but every SSH operation still checks the host pin.

For an already enrolled device, the comma-side management shell is:

```sh
python3 /data/openpilot/tools/jetlink/manage_jetson.py shell
```

To disable only Internet sharing, remove the opt-in marker and stop
`hl-jetson-internet.service`. Its cleanup removes only its own chains and
restores forwarding. This does not disable the existing USB inference channel.

## Validation

- 94 focused desktop tests pass, including native configfs NUL-terminated MAC
  fields, the comma's pipe-separated DNS output, DHCP-preserving route setup,
  peer authentication failure, NAT restrictions, cleanup/crash recovery,
  idempotent profile writes and old/new Navdy endpoint selection.
- 21 routing tests pass on the actual comma, using an isolated test runtime
  under `/tmp`, without installing packages into openpilot's Python environment.
- Actual comma-to-Jetson SSH succeeds with the comma's own private key.
- Jetson HTTPS HEAD to the selected signed bundle returns 200 through the
  comma; the 64,518,963-byte bundle downloads and stages successfully. Existing
  model cache is retained and checked by the unchanged updater.
- The source commit selected by the tested comma is
  `84087a5b78118acc40234bfd8ef64235421f1aab`; no newer model is substituted.
- The unchanged signed updater subsequently reports `applied`/`ready`, with
  that commit at `current/SOURCE_COMMIT` and a readiness record matching the
  current Jetson boot ID. The inference service is active. Fresh comma-side
  telemetry reports `active: true`, `ready: true`, Cinque v2 and no error.
- Activation re-enumerates the USB link and changes the Jetson DHCP address
  from `192.168.60.12` to `192.168.60.218`. The sharing daemon authenticates the
  same pinned host at its new address and restores routing automatically.
  A subsequent Jetson HTTPS request returns 200 and NTP reports synchronized.
- During download, a 20.007-second native observation receives 400 modelV2 and
  400 driverStateV2 messages, all valid with consecutive frame IDs. Vehicle
  remains Park, speed zero, disengaged. This is not road validation or a strict
  greater-than-or-equal-to-20-Hz acceptance measurement.
- After activation, a 30.000872-second native observation receives 600 modelV2
  and 601 driverStateV2 messages, all valid with consecutive frame IDs. Fresh
  telemetry still reports Jetson inference active; Park/zero/inactive remains.
  The comma's paired SSH independently resolves the changed USB address and
  returns the expected Jetson hostname using its own key.
- Only the Navdy bridge and new routing service have been restarted. Existing
  model/DM process IDs remain unchanged. No full comma/Jetson reboot is performed.

Navdy remains absent from the Jetson USB and ADB inventories. Kernel USB logs
show comma re-enumeration but no Navdy attachment. Its cable, connector, port
and device USB configuration still require isolation; this evidence does not
identify which one failed. Physical Navdy communication and a full reboot of
the newly installed sharing setup remain unverified. No road-validation claim
is made from stationary inference or network checks.

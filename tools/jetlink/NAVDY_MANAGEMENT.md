# Navdy management on a protected Jetson

This optional standalone installation lives at `/run/carrot-data/hl-navdy`.
That path is the persistent DATA partition, despite its `/run` prefix.
It does not change the read-only root filesystem, boot gate, inference runtime,
USB descriptors or startup services. It is not installed by a Git pull alone.

## On the prepared Jetson

```sh
sudo python3 /run/carrot-data/hl-navdy/navdy_management.py status
sudo python3 /run/carrot-data/hl-navdy/navdy_management.py devices
sudo python3 /run/carrot-data/hl-navdy/navdy_management.py adb --serial USB_SERIAL shell getprop ro.product.model
sudo python3 /run/carrot-data/hl-navdy/navdy_management.py stop
```

`devices` starts a separate ADB server bound to localhost port 5038. No ADB
server starts at boot. Its private identity is kept in `state/.android` on DATA,
not the image's volatile home overlay. Do not delete or publish that directory.
The selected device must be authorized USB hardware with `com.navdy.hud.app`.
Authorize the connection on Navdy only after checking the displayed identity.
An empty device list is not a successful Navdy connection.

`adb --serial` also accepts `pull`, `push`, `install`, `uninstall`, `logcat`,
`get-state`, `reboot`, `root` and `remount`. Device firmware may deny root or
remount; an available command does not prove support. Stop the display bridge
before APK replacement or device restart and verify recovery while parked.
TCP ADB discovery and externally listening ADB servers are not exposed.

`relay` runs the repository's Navdy JSON/USB transport on SSH stdin/stdout.
It is an optional endpoint, not an automatically running bridge. The Comma
bridge must explicitly select this endpoint before display/navigation traffic
uses it. The existing `/opt/carrot-jetlink/current/...` bridge target is not
silently replaced. Avoid running two bridges against one Navdy.

The installation manifest checks payload ownership, permissions and SHA256
hashes before execution. It is a local integrity record, not a signed upstream
release. The portable ADB packages are extracted from authenticated Ubuntu ARM64
packages without installing packages into the protected OS.

## Later, with the actual Comma available

1. Follow [REMOTE_ACCESS.md](REMOTE_ACCESS.md) to enroll the Comma's own public
   SSH key, verify the Jetson host key, and configure USB management offroad.
   Never transfer a private key from the Mac to Comma.
2. The prepared `usb_management.py` is a copy of the host-profile installer.
   Run its `install-host` action only when the real Comma NCM interface exists.
   It deliberately refuses to invent a USB interface or overwrite foreign ones.
3. Verify Comma-to-Jetson SSH over USB with Ethernet and Wi-Fi removed. Mac-to-
   Jetson USB SSH at `192.168.55.1` is a different USB role and is not that test.
4. Attach Navdy to Jetson USB-A, authorize ADB, verify device/package identity,
   and test actual display information and phone-to-Navdy navigation return data.
5. Wire the Comma bridge to this endpoint (or a supported managed runtime),
   then validate coexistence with real inference and reconnection before use.

Only Comma needs Tailscale for the SSH jump arrangement, but Tailscale sharing
alone does not grant SSH access. USB management and inference share hardware;
USB resets can interrupt both. Keep an independent recovery path for updates.

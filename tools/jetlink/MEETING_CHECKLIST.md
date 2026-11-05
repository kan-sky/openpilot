# Jetson Maintenance Field Checklist

This is a test deployment, not a field-validated guarantee. Use the recipient's
actual comma, carrot-jetson R2 device, SSD, TURZX and usual phone hotspot. Never
substitute the separately connected development Jetson. Work parked with the
ignition off. Do not start driving until installation or recovery finishes.

## Before Meeting

- Publish and install the reviewed `cluster_jetson` commit on the intended comma.
  Uncommitted tracked changes and other branches are refused by the sync tool.
- Bring the Mac, NVMe enclosure, data USB cables, both devices and their power.
- Confirm existing access to the comma. This kit does not bypass comma SSH login.
- Keep a Mac-only administrative SSH key as a second recovery path. Its private
  key stays on the Mac; the comma has a different device-specific private key.
- Collect all existing Jetson public keys that must retain access. `CARROTSETUP`
  enrollment replaces the authorized list; previously consumed setup files do
  not reveal every installed key. Stop rather than unknowingly revoke access.
- The current installer targets R2. Boot-gated/new images and incomplete DATA
  recovery are rejected. Do not disable those checks to make installation pass.

## 1. Prepare Access

Run on the intended comma, through its existing authenticated terminal:

```sh
cd /data/openpilot
python3 tools/jetlink/manage_jetson.py init-key
```

Only export `/data/hl-jetson/id_ed25519.pub` to the Mac. Never export the private
key. The persistent directory is outside the Git checkout; Git pull does not
replace the pairing, key, host identity or job records.

Generate or select an administrative Mac key (not the comma key), preserving any
existing key. Export only its `.pub`. Shut down Jetson, disconnect its power,
and attach its SSD to the Mac. Reject format/initialize dialogs. Do not mount
or modify Linux DATA or the immutable system image.

On Mac, in the extracted meeting kit, inspect first:

```sh
python3 prepare_ssd.py --public-key /path/to/comma.pub --public-key /path/to/admin.pub
```

Review the reported `CARROTSETUP` device, public-key count and preservation of
existing Wi-Fi settings. Supply additional `--public-key` arguments for every
key to retain. Only after confirming the full list, write and read back:

```sh
python3 prepare_ssd.py --public-key /path/to/comma.pub --public-key /path/to/admin.pub --write --existing-keys-complete
```

Safely eject the SSD, reinstall it in the unpowered Jetson, then boot. Saving
setup.json is not enrollment success: verify `SETUP-RESULT.json` when accessible
and an actual SSH login. The SSD operation alone cannot install the runtime.

## 2. Pair the Actual Devices

Turn off comma USB debugging (`AdbEnabled`) through the normal setting before
using Jetlink USB. AGNOS's ADB/NCM gadget `g1` otherwise owns the controller;
Jetlink deliberately refuses to unbind an unrelated gadget. Verify Wi-Fi SSH
access first. Disabling ADB does not disable ordinary Wi-Fi/Tailscale SSH.

Connect both devices to the recipient's usual hotspot. Verify comma can reach
Jetson, not just that the Mac can reach each device. Hotspot client isolation can
block peer traffic. Confirm the actual Jetson address before accepting its first
SSH host fingerprint on the Mac; compare with a trusted device-side fingerprint
where available. Never use `StrictHostKeyChecking=no` or auto-accept a changed key.

Over that verified Mac-to-Jetson SSH session, obtain the public host key:

```sh
cat /etc/ssh/ssh_host_ed25519_key.pub
ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub
```

Copy only this public host key to `/tmp/jetson-host.pub` on comma. Run on comma
with the actual IP and the independently checked `SHA256:...` fingerprint:

```sh
cd /data/openpilot
python3 tools/jetlink/manage_jetson.py pair --host 192.168.1.123 --host-key-file /tmp/jetson-host.pub --fingerprint SHA256:REPLACE_WITH_VERIFIED_VALUE
python3 tools/jetlink/manage_jetson.py shell
```

`pair` verifies the key, the `jetlink` login and passwordless sudo. A mismatch
fails closed. No shared password, public listener or unauthenticated shell is
installed. The private device key intentionally has no passphrase so the service
can operate; it and pairing records are protected by directory/file permissions.
Exit the test shell before continuing. Automatic updates are still disabled.

## 3. First Install and Real Output

Keep ignition off and assistance disengaged, with stable power. Leave USB
connected so current device state/address discovery is available. Unlike the
old manual Windows installer, this management worker supervises the installation
through a separate LAN lease and stops/restarts Jetson inference/HUD services.

Run on comma:

```sh
python3 tools/jetlink/manage_jetson.py sync --confirm-parked
python3 tools/jetlink/manage_jetson.py inspect
```

The engine probe can take about 20 minutes. `succeeded` means the runtime probe
and three service-active samples passed, not that USB pixels were observed.
Connect TURZX to Jetson. Set comma `ClusterHud` to the actual panel type and
`ClusterHudScreenMode=0`. Normal display requires ignition/onroad; perform that
stationary display check only after maintenance is finished. Verify speed/gear,
clock, icons, music/cover and BSM using known real inputs or the existing debug
mode. Exit debug mode before use. This maintenance feature changes no CAN or
vehicle-control logic.

## 4. Recovery Rehearsal Before Enabling Auto

Complete all checks while stationary; no road test is required for maintenance.

- Reboot both devices, verify `shell`, `inspect`, USB inference and actual TURZX.
- Reconnect the usual hotspot with a changed Jetson IP. Current USB health
  telemetry supplies candidate addresses, but the SSH host key must still match.
  If telemetry is unavailable, previous/manual IPs are tried; no subnet scanning
  or unconditional rediscovery is promised. After verifying the same device,
  run `python3 tools/jetlink/manage_jetson.py set-host --host NEW_IP` on comma.
  This verifies the same pinned SSH identity before saving the new address.
- Run an unchanged `sync --confirm-parked`: expect `unchanged`, no runtime switch.
- Test `rollback --confirm-parked` and then `sync --confirm-parked --retry`.
  The first rollback restores stock; later rollbacks restore the preceding custom
  version. Confirm both service status and actual output after restoring custom.
- Test `cancel` from a second comma terminal during a controlled update; expect
  `rolled-back` or `cancelled`. Do not remove power to simulate a fault.
- Simulate loss of LAN only while parked. Missing/revoked offroad lease aborts
  the worker and attempts recovery. Reconnect, `inspect`, then `sync` to collect
  the same job result. It must not dispatch a duplicate installation.

The host lease expires after 20 seconds without renewal. The client reads fresh
deviceState plus explicit IsOffroad/IsOnroad flags; missing data is not permission.
Rechecking and rollback are not instantaneous. Keep the vehicle off until done.

After every check passes, run on comma:

```sh
python3 tools/jetlink/manage_jetson.py auto on --confirm-field-checks
```

Thereafter the manager checks after 30 seconds of fresh continuous offroad state.
Following a Git pull/reboot, it packages the committed cluster sources/assets,
verifies the paired host, skips matching content, and applies changed content.
Unhydrated Git LFS assets, dirty tracked files and unsupported images are refused.
Failed content is not retried automatically; inspect and resolve it before an
explicit `sync --confirm-parked --retry`. Rollback/recover disable automatic mode.

## Remote Support and Faults

Commands below run on the comma; the user need not carry a Jetson SSH key:

```sh
python3 tools/jetlink/manage_jetson.py inspect
python3 tools/jetlink/manage_jetson.py auto off
python3 tools/jetlink/manage_jetson.py cancel
python3 tools/jetlink/manage_jetson.py rollback --confirm-parked
python3 tools/jetlink/manage_jetson.py recover --confirm-parked
```

`inspect` includes a bounded tail of the latest installation log. Local state
and last results live in `/data/hl-jetson`; host jobs live under
`/var/tmp/hl-maintenance`. Review logs for device identifiers/IPs before sharing.
Never share the whole `/data/hl-jetson` directory: it contains the private key.
`auto off` prevents future jobs; use `cancel` to stop an existing job.

If dispatch status is ambiguous, use `inspect` then `cancel`; cancellation fences
off even a delayed start request. If power loss left `recovery_required=true`,
collect/resolve the old job first, then run `recover` while offroad. The persistent
transaction restores the previous selection/updater. Do not erase the journal,
pin, or original release. Recovery that cannot restart services retains evidence
and reports `needs-attention`; it never declares success unconditionally.

Keep the administrative Mac key available separately. This is LAN maintenance
through comma, not an Internet remote-access VPN. Direct support from a distant
Mac still needs an independently authorized, tested route to the comma. Git pull
alone does not provide that route. OS boot failure, corrupt storage, power loss
or loss of every network path can still require local physical help.

Old runtime releases/jobs are retained, not silently deleted. Less than 512 MiB
free blocks new installs; inspect disk use and intentionally archive/remove old
unused releases only after preserving the selected and recovery versions.

## Field Result Record

Record comma commit, Jetson image version, overlay content hash, verified host
fingerprint, usual-hotspot reachability, reboot/IP-change/rollback/cancel results,
and photographs of real TURZX output. Do not record private keys or Wi-Fi secrets.
Leave automatic updates disabled if any required check fails.

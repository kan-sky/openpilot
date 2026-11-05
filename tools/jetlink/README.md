# Jetlink Maintenance

## Current Package Layout

The maintained carrot package uses `openpilot/...` inside the repository and
host release. The selected Jetson runtime is `/opt/carrot-jetlink/current`,
not `/opt/hl-cluster/current`. Native inference, model selection and the
existing signed host-update pipeline remain upstream-owned.

The overlay builder now emits manifest format 2 with `package_layout=openpilot`
and `hud_capability=carrot_hud_v1`. Legacy format-1 / `hl_cluster_v1` bundles
are rejected. The installed base must already contain the new package layout;
this overlay does not migrate a legacy image or change AGNOS/JetPack versions.
Sync accepts only clean, committed `WIP-FR` or `cluster_jetson` sources. Detached
HEADs, temporary `codex/*` branches and other branches remain blocked, including
if the branch changes while packaging. Automatic sync still requires explicit
field-check enrollment; adding a branch does not enroll or pair any device.

Both host bundle builders include `openpilot/selfdrive/navdy`. The standalone
relay command is:

```sh
/opt/carrot-jetlink/venv/bin/python -u /opt/carrot-jetlink/current/openpilot/selfdrive/navdy/transport.py --stdio
```

Use only the existing paired SSH configuration and pinned host key. No automatic
pairing is performed. The overlay installer requires `adb` on the Jetson and
checks the relay's `--help` without enumerating or starting a USB device. Install
the Jetson's `adb` package explicitly before applying a relay-bearing overlay.
USB permissions for the SSH user, Navdy authorization and actual relay operation
still require the connected target devices; these checks are not field validation.

USB management is opt-in and latched once per OS boot. Only explicit `usb` mode
can add it, while the existing fresh offroad/ignition-off/driver-view guard permits
setup. Auto/Android/iOS remain unchanged. The daemon rechecks offroad state before
post-bind network activation; network failures leave bulk inference intact. Cleanup
detaches only after native owner shutdown and verified UDC unbinding; unverified
descriptor rollback or native cleanup blocks rebinding. Existing
paired-key and service-restarting-update enrollment gates remain required; no
automatic pairing or hot setting application is introduced. Physical composite
USB and SSH continuity still require target-device validation.

## Legacy Enrollment Reference

The procedures below originated on the separate `cluster_jetson` branch. Their
pairing, rollback and safety gates are retained, but earlier device validation
does not establish compatibility of the new package, protocol or Navdy relay.

This branch adds Jetlink inference to the existing fix5 comma code and runs its
original 1920x480 ambient instrument cluster on a Jetson-connected TURZX USB
display. Car control, CAN, driver monitoring and CarrotLink remain on comma.

## Target

- Install the Jetson using <https://github.com/ajouatom/carrot-jetson> first.
- The host overlay targets its R2-style `/opt/carrot-jetlink` installation:
  `current -> releases/...`, prepared `venv`, model cache, updater and NVIDIA Xorg.
- The image's kernel, QSPI, TensorRT, engine cache, networking and protected
  storage configuration are reused, not reflashed or replaced.
- A generic JetPack installation is not the target. The separately connected
  Orin NX development machine has not been installed or modified.
- Newer boot-gated images with `boot-update-required`, incomplete DATA recovery,
  or pending update transactions are rejected rather than guessed compatible.

**Installing this comma branch alone does not change a stock Jetson's UI.** The
matching host overlay below is also required for the additional display assets.

## Data Path

```text
CarrotLink phone -> comma phone_media.json (existing endpoint unchanged)
comma cereal -> existing OpenpilotLiveSource -> ClusterUiState
  -> bounded Jetlink USB display packets -> Jetson RemoteLiveSource
  -> existing ClusterUiRenderer -> JPEG -> TURZX USB monitor

comma cameras -> QCOM image warp -> Jetlink USB -> Jetson TensorRT
  -> validated model output -> comma modeld -> existing planners/controllers
```

- Wire capability is `carrot_hud_v1`; legacy `hl_cluster_v1` is incompatible.
  Both ends must run matching reviewed sources. Phone artwork is an optional
  bounded extension of the existing snapshots, with SHA256-checked assembly.
- No image re-encoding or UI drawing runs on comma modeld's thread. The display
  producer is a separate low-priority process. Host display errors do not
  invalidate inference results.
- Missing/stale snapshots expire after 500 ms. Speed/BSM/music return to standby;
  the output gate then shuts the panel off, at its normal one-second polling
  interval. A disconnected link never leaves indefinitely frozen driving data.
- The original Korean font, title scrolling, cover decoding, BSM fade, icons,
  and 12-hour KST clock are reused. The Jetson derives display time from comma;
  it does not set either machine's system clock.
- Theme is still selected on comma. Dark = 20%, light = 60%; automatic theme
  uses comma's KST clock. The old manual brightness setting remains ignored.
- JPEG output (encoding setting 1) at 10 FPS is the initial host configuration.
  It does not attempt to use comma's Qualcomm hardware H264 encoder on Jetson.
- Route video/navigation debug image streaming is not ported. Direct CAN
  corner-radar debug parsing is disabled in this display producer. The ambient
  BSM indicators still use carState's existing left/right blindspot values.

## Follow Comma Wi-Fi

The existing `carrot_wifi_v1` USB channel sends comma's saved client networks,
with its currently connected Wi-Fi first and marked active. The installed
carrot-jetson Wi-Fi worker imports them and attempts that active network before
fallbacks, even if Jetson is already connected to an administrator-added Wi-Fi.
No SSH pairing or Internet connection is needed for this USB provisioning.

- Comma refreshes the list every 30 seconds; Jetson checks about every 30 seconds.
  A changed Wi-Fi can therefore take about a minute plus association/DHCP time.
- Personal WPA/WPA2-PSK, WPA3-SAE and open client profiles are supported. Comma's
  own hotspot/AP profile and enterprise authentication are not exported.
- Unreadable inactive profiles are skipped. If the current client's credentials
  cannot be read, no replacement is published until a successful retry.
- Passwords use only the private USB channel and root-private NetworkManager
  storage, never HUD snapshots or diagnostic logs. Physical attachment grants
  the connected Jetson access to these saved client credentials.
- Jetson's protected-storage helper persists successfully applied profiles.
  Manually added Wi-Fi and Ethernet profiles are preserved. Unavailable Wi-Fi
  may fall back to another saved network; the same network still needs to be
  reachable by Jetson, and hotspot client isolation can prevent peer SSH.

## Build the Host Overlay

On a machine with this checkout and Python 3.10 or newer:

```sh
python3 tools/jetlink/build_cluster_bundle.py tools/jetlink/dist/cluster-jetson-overlay.tar.gz
```

The builder includes only selected runtime sources and tracked display assets,
not phone logs, credentials, ONNX files or local previews. Git LFS pointers are
rejected: fonts/icons must be hydrated first. A manifest records every file's
SHA256, the base commit, and a content ID, including any working-tree edits.
The `.sha256` file contains the digest needed by the installer.

## Install on the Intended Jetson

### Paired Comma Maintenance

For a recipient who cannot repeatedly bring the SSD back, the opt-in LAN SSH
maintenance path pairs a device-specific comma key with the intended Jetson.
When an already configured USB management interface passes the exact route/MAC
checks, it is tried before LAN; see the legacy [USB-first remote access](REMOTE_ACCESS.md).
Provisioning requires explicit `usb` mode, the boot-latched opt-in and fresh offroad state.
The user does not handle keys during routine updates. After the field checklist
passes and `auto on --confirm-field-checks` is explicitly set, new committed `WIP-FR` or `cluster_jetson` sources
can be synced from comma during verified offroad periods. This is not keyless
authentication. Tailscale is installed and enrolled separately on comma only.

Read [the complete field checklist](MEETING_CHECKLIST.md) before first enrollment.
It includes Mac SSD preparation, host-key verification, manual sync, real display
checks, rollback/cancellation rehearsal, and only then automatic enrollment.
No connected development Jetson is used by the build/tests.

```sh
python3 tools/jetlink/build_meeting_kit.py tools/jetlink/dist/HL-Cluster-Meeting-Kit.zip
```

The kit contains no keys, Wi-Fi settings or device addresses. Its management
client belongs in the matching comma checkout, not as a standalone Mac daemon.
Mac SSD preparation only registers public keys; the booted devices are still
required for pairing, installation and verification. Existing Windows/manual
installation remains supported. Old Windows rollback restores upstream; the new
maintenance rollback restores the previously selected version.

### Windows Helper

Build the recipient's ZIP (also rebuilds the matching overlay):

```sh
python3 tools/jetlink/build_windows_helper.py tools/jetlink/dist/HL-Cluster-Windows.zip
```

Send the ZIP, not a private SSH key. On Windows 10/11, extract everything and
double-click `Start.cmd`. No Python installation on the PC is needed. The Korean
menu includes SSH key enrollment preparation, installation, last-job inspection,
and rollback. Windows OpenSSH Client must be installed. Host-key checks and
existing key passphrases remain enabled; system execution policy is not changed.

First-time enrollment still needs the powered-off Jetson's SD/SSD mounted on the
PC as exactly one `CARROTSETUP` FAT volume. The helper generates a PC-local key,
preserves existing setup settings, and warns that keys absent from the enrollment
list lose access. It never formats disks. Writing setup.json is only preparation:
boot and successful SSH login establish enrollment. This follows the upstream
[SSH enrollment contract](https://github.com/ajouatom/carrot-jetson/blob/main/docs/JETSON-SSH.md).

After checksum and target checks, installation runs as a Jetson systemd job,
independent of the PC connection. Query its result from menu 3 after reconnecting.
The helper keeps job identifiers under `%LOCALAPPDATA%\HLCluster`; do not retry
blindly after a lost reply. Jobs/logs are retained under `/var/tmp/hl-cluster-*` on
Jetson. Neither machine should be powered off mid-install. A successful job means
installer/probe and service checks passed, not that USB pixels were physically
verified. The matching comma branch must still be installed separately.

This is a test package: PowerShell core logic/parser and local mocked installation
tests can run on macOS, but Windows PowerShell 5.1 console/file-picker behavior,
the actual Windows SSH client, FAT media, and the target Jetson need field testing.

After extracting the generated ZIP, the portable helper tests can run in
PowerShell (both tests use local fixtures; menu SSH/SCP are mocked):

```powershell
pwsh -NoProfile -File tools/jetlink/windows/Test-Core.ps1 -PackageRoot <extracted-HL-Cluster-Windows-folder>
pwsh -NoProfile -File tools/jetlink/windows/Test-Menu.ps1 -PackageRoot <extracted-HL-Cluster-Windows-folder>
```

Use `powershell.exe` instead of `pwsh` for a Windows PowerShell 5.1 check. These
developer test scripts are not included in the recipient ZIP.

### Manual SSH

Park the vehicle, disengage assistance, disconnect comma from Jetson, and use
the image's documented SSH public-key enrollment. Keep the original SD/card
available as a recovery option. Copy the bundle and this branch's
`tools/jetlink/install_cluster.py` to that Jetson, for example under `/tmp`.
Use **that device's real IP**, not the unrelated `jetson-nx` development alias.

```sh
python3 /tmp/install_cluster.py check
sudo python3 /tmp/install_cluster.py install \
  --bundle /tmp/cluster-jetson-overlay.tar.gz \
  --sha256 <digest-from-the-sha256-file> --confirm-parked
```

The installer:

1. Checks the installed image/runtime/storage and acquires the updater lock.
2. Stops inference and HUD services; stages a copy of the current release.
3. Verifies the bundle and overlays only our server/cluster sources and assets.
4. Uses the image's Python environment for imports, and its native engine probe
   for model-load and finite synthetic-inference checks. A failed probe keeps
   the previous selection and restarts the previous services.
5. Atomically selects the new release. Saves the previous release and updater.
6. Pins the custom runtime: the upstream stage/activate updater becomes a no-op
   while `HL_CLUSTER_PIN.json` exists. This prevents a reboot/update silently
   replacing our UI. No update signature check is weakened or bypassed.
7. Restarts the existing services, retaining the original Xorg, USB ownership
   lock, Wi-Fi provisioning, protected DATA mount and recovery boot display.

The installer deliberately refuses to overwrite an already pinned installation.
For another overlay, rollback first, then install the verified bundle. Reinstalling
the same bundle after rollback creates a fresh, probed release and preserves the
previously retained copy without reusing or overwriting it.
Do not manually delete the pin or change `current`.

On comma, install `cluster_jetson`, reboot offroad, keep Jetlink transport `usb`
(the default), set `ClusterHud` to the actual panel type (1: 9.2-inch, 2: 12.3-inch)
and `ClusterHudScreenMode=0` for the ambient display. `ClusterHudDebug=0` requires
ignition/onroad; the existing debug modes allow a stationary display check.
Connect comma's data port to a Jetson host USB port, and the TURZX to another
Jetson host port. SuperSpeed is required for the Jetson inference link.

## Rollback

With the vehicle parked and comma disconnected:

```sh
sudo python3 /tmp/install_cluster.py rollback --confirm-parked
```

This restores the saved runtime selection and original updater, then restarts
the existing services. Neither original nor custom release directories are
deleted. Upstream signed updates resume. For loss of power during installation,
retain the card and installer logs: do not erase updater backup/pin files to
force another attempt. Runtime selection uses atomic replacement, but actual
power-loss recovery has not been hardware-tested.

## Validation and Diagnosis

The comma onroad UI displays the existing Jetlink status on both UI variants:
upper right on comma 3/3X, beside the gear indicator on comma 4. The compact
badge is hidden while a driving alert is displayed.

- Yellow `jetSON WAIT`: connecting or loading (`Jetlink WAIT` before peer identity).
- White `jetSON READY`: the host is ready, but modeld is not using it yet.
- Green `jetSON`: modeld reports active remote inference.
- Red `jetSON ERROR`: a reported model/link/health fault or stale link status.
- Yellow `jetSON HOT` / `jetSON DATA?`: a health warning / missing current health data.
- No badge: no host has been detected, or a fresh waiting report confirms absence
  with no active model or model failure.

These are inference-link indicators, not confirmation that TURZX received or
displayed a frame. The UI polls status every 250 ms; malformed or missing status
files must not crash the driving UI. Use the separate HUD heartbeat below for
display-transport diagnosis.

WAIT/ERROR also show the reported preparation stage/progress/message, model
error, link error or host-health reason. Badge and explanation use the same
snapshot; stale historical errors are not presented as current causes. When
there is no specific message, the UI states the observed phase or missing/stale
status rather than guessing a cable or power fault. Long explanations wrap and
cycle through pages every four seconds, restarting only when the status changes.
READY/active clears the explanation. On comma 4 the details stay in the narrow
column beside the gear indicator and are hidden along with the badge for alerts.

```sh
# comma
cat /dev/shm/carrot-jetlink.json
cat /dev/shm/carrot-jetlink-model.json

# Jetson
readlink /opt/carrot-jetlink/current
systemctl status carrot-jetlink carrot-jetlink-hud carrot-jetlink-xorg
journalctl -b -u carrot-jetlink -u carrot-jetlink-hud --no-pager -n 150
lsusb
cat /dev/shm/hl-cluster-status.json
```

`ready` in jetlinkd is not proof that modeld is using Jetson: check `active` in
the model status as well. Entry uses the upstream stopped/physical-steering-
override gate and fresh car state. Timeouts, malformed outputs and model
contract mismatches fall back to the warmed local model; an active-link failure
also raises the existing commIssue disengagement event. This is not permission
to validate experimental vehicle-control code by driving on public roads.

The HUD heartbeat is written only after a successful JPEG send, not merely
when the renderer starts. It cannot prove correct pixels on a physical panel.
Do not share Wi-Fi packet files, NetworkManager profiles or phone-media dumps;
these may contain private data. The normal timing/status files are preferable.

Local regression commands (run with the repository's Python dependencies):

```sh
PYTHONPATH=.:tools/jetlink python3 -m pytest --confcutdir=tools/jetlink -o addopts='' tools/jetlink -q
PYTHONPATH=. python3 -m pytest --basetemp=/tmp/hl-model-tests --confcutdir=selfdrive/modeld/tests \
  -o addopts='' selfdrive/modeld/tests -q
PYTHONPATH=. python3 -m pytest --confcutdir=selfdrive/ui/tests -o addopts='' selfdrive/ui/tests/test_jetlink_badge.py -q
PYTHONPATH=.:tools/jetlink python3 tools/jetlink/preview_cluster.py tools/jetlink/dist/preview
```

The preview executes the real raylib renderer and compares direct vs transported
state pixel-for-pixel at 1920x480, in both themes. Its green album cover is a PNG
decode fixture, not a real song's album artwork. This verifies layout/transport,
not Jetson FPS, QCOM GPU warp, USB timing, actual panel brightness or vehicle
behavior. Those require the intended carrot-jetson device, comma and TURZX.

## Source Baselines

- comma Jetlink adapter: ajouatom/openpilot `carrot-wip`
  `335c1c2ffc2477214f6f822c310018d7cbe6516c`, adapted to this older source layout.
- Image/runtime contract inspected: ajouatom/carrot-jetson
  `d0dc96fa32410403dfb2fefff3e2909c11ec109b`.
- Vendored transport attribution: `third_party/jetlink/UPSTREAM.md` and LICENSE.
- No general upstream merge, AGNOS update, CarrotLink APK change or change to
  `fix5` is included. The custom branch has not been hardware-qualified.

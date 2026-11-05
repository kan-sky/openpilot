# USB-first remote access

```text
Mac -> Tailscale -> comma sshd -> Jetson sshd
                                 1. USB management (169.254.77.1)
                                 2. same hotspot/private LAN

comma FunctionFS bulk interface 0 -> Jetlink inference (unchanged protocol)
comma additional CDC-NCM interface -> SSH management
```

Only comma needs Tailscale and an Internet connection. Jetson does not need its
own VPN or Internet connection for USB SSH. This is an SSH jump-host arrangement,
not an exit node, advertised LAN route, bridged Wi-Fi, or USB role reversal.
Jetson remains the USB host. The existing C-to-C host policy stays installed.

USB management is opt-in because the actual comma kernel must support CDC-NCM
and the extra composite endpoints. Configuration
failure records `/dev/shm/hl-jetson-usb.json` and leaves the bulk-only gadget when
cleanup succeeds. No live gadget is unbound to enable the feature. Enable it only
offroad and verify SuperSpeed, inference and TURZX before driving. Unexpected
enumeration/bind failure requires disabling USB management and rebooting offroad;
passing unit tests is not physical compatibility validation.

NCM and FunctionFS share a USB controller. An inference transport reset or UDC
rebind also interrupts USB SSH; this is not a second physical cable. Do not
assume USB-only software updates survive an inference-server restart. Prove
that field case before enabling automatic rollout; the existing offroad lease
and rollback stay fail-closed, and same-hotspot SSH is the recovery option.
By default `sync`, `rollback` and `recover` use LAN even when ordinary SSH chose
USB. With no LAN they refuse the update, not the diagnostic shell. Only after
proving service restart, reconnection and recovery on the actual pair, run
`manage_jetson.py usb-updates on --confirm-field-checks` to qualify USB updates.

## One-Time Setup

First enable USB management on comma (below), disable ADB, and reboot offroad
with the Jetson USB cable attached. Network configuration runs after FunctionFS
binds the controller: AGNOS 4.9 exposes NCM interface/MAC attributes only then.
Kernel-generated MAC addresses are supported; fixed MACs are not required.

On the prepared carrot-jetson host, via its already authenticated LAN SSH account:

```sh
sudo python3 /tmp/usb_management.py install-host
```

Copy this repository's `tools/jetlink/usb_management.py` to that path first. This
adds only `hl-jetson-usb.nmconnection` and invokes the image's protected identity
save. Existing Wi-Fi profiles and keys are preserved. The installer requires one
CDC-NCM interface under USB `1209:0001`, revision `0102`. Its profile matches the
exact USB port path plus `cdc_ncm` driver, uses `169.254.77.1/30`, never supplies a
default route, and survives the image's volatile `/etc` overlay. No SSD reflash,
root-filesystem remount, upstream service replacement, or inference restart is
needed. Remove this profile and run `protected_storage.py save` to undo it.
Keep using that Jetson USB port (and hub topology). Moving the cable to a different
port requires reviewing/removing this profile and running `install-host` again
over LAN. An exact legacy fixed-MAC profile migrates automatically; customized
profiles are refused rather than overwritten.

On the intended comma checkout, with ignition off:

```sh
python3 tools/jetlink/manage_jetson.py usb on
```

This creates a comma-specific SSH key if missing and enables the new descriptors
for the next offroad reboot. A root-owned `/run/hl-jetson-usb-mode` latch keeps
that choice unchanged across manager restarts and USB reconnects in the same
boot. A damaged latch disables optional management and reports the error;
bound/foreign gadget ownership checks still refuse unsafe descriptor changes.
The comma address is `169.254.77.2/30`. Export only
`/data/hl-jetson/id_ed25519.pub`, append it to the Jetson's existing authorized
keys using the already authenticated maintenance connection, then run the
Jetson's `/usr/lib/carrot-jetlink-storage/protected_storage.py save` through sudo.
Never copy a private key into Git, an update bundle, or the other device.

Pair using the independently verified Jetson Ed25519 host key and fingerprint:

```sh
python3 tools/jetlink/manage_jetson.py pair --host 169.254.77.1 \
  --host-key-file /data/hl-jetson/host.pub --fingerprint SHA256:<verified-fingerprint>
python3 tools/jetlink/manage_jetson.py shell
```

The existing LAN pairing procedure works too. Once paired, management tries a
direct USB route with the correct source address, owned NCM function/MAC and carrier before trying
fresh USB-reported LAN addresses and saved LAN addresses. A stale USB address is
never sent through Wi-Fi's default gateway. Host-key pinning applies to both
paths. First pairing and real SSH login are mandatory; an IP is not identity.

## Tailscale On Comma

### Carrot Web Setup

On the same trusted Wi-Fi/hotspot, open `http://<comma-LAN-IP>:7000`, then
Settings -> Device -> Remote Support ("원격지원"). Install Tailscale if absent,
press Login, and open the displayed official login link using the device
owner's account. Installation alone is not enrollment. Account Change explicitly
logs out the displayed account before creating a new login link; old VPN sessions
disconnect and the new owner must share the device again. Existing supported
HL, Wayon and system installations are detected instead of installing a second
daemon. A routine status check or Login on an already running account never
logs it out. A stopped or unfamiliar installation may need service repair first.

The Tailscale Connection switch resumes the saved connection using `tailscale up`
with no flags or pauses it using `tailscale down`. It does not log out,
delete keys, change DNS/routing preferences, stop the daemon or reset Jetson USB.
The stopped state survives normal daemon/comma restarts. Turning it off interrupts
Tailscale remote access; open this LAN-only page on the same trusted hotspot to
turn it back on. An expired or missing login requires the separate Login action.
The helper bounds CLI execution externally: even `up --timeout=5s` is treated as
a settings change by Tailscale and can reject existing non-default DNS settings.

Remote-support actions do not require offroad state. They do not restart Jetlink,
reset USB, change the driving model or reboot comma. Account changes can interrupt
remote management. CLI helpers use normal scheduling, CPU 0-1 and nice 10;
the USB owner's FIFO1 priority is limited to an active inference IPC session.
These priorities reduce contention but are not a hard realtime guarantee. Operate
the web page as a passenger or while stopped, not as
the driver. Setup is deliberately LAN-only, with same-origin and per-server CSRF
checks. It is not an authenticated multi-user administration portal: use a trusted
hotspot, never port-forward Carrot Web to the Internet. VPN clients can use SSH;
the account-changing API is not exposed to Tailscale source addresses.

The support operator's Ed25519 **public** key can be registered after comparing
its SHA256 fingerprint. This enables ordinary SSH and preserves existing owner
keys. Only keys added with the support marker can be removed from this panel.
Tailscale node sharing is still completed in the owner's Tailscale machine page;
sharing alone does not authorize SSH. Replacing/removing GitHub SSH keys through
the existing Developer or native SSH screen also replaces this shared key store;
verify support access afterwards. Never paste a private key or an auth key here.

Settings -> Device -> Jetson shows the same fresh status used by the native comma
badge: READY, WAIT, ERROR and details, model identity, actual inference activity,
temperature, LAN IPs, protocol/runtime, and reported physical HUD connectivity.
READY means the model is prepared, not that driving inference is currently active.
Link/model/health data older than three seconds cannot report current health.
The SSH button performs a real pinned-host check, USB first then LAN; its result
is labelled as the last check, not a continuous connection measurement.

Jetson settings expose the existing `ClusterHud` and `ClusterHudTheme` Params and
the optional USB-management marker. USB changes are saved for the next parked
comma reboot without resetting the live link. First Jetson key enrollment and
host-key pairing are still required; this page never trusts an unknown SSH host
automatically. No model compilation, power-mode change or automatic provisioning
is triggered by opening the page.

The opt-in installer targets only ARM64 AGNOS comma hardware. It downloads the
official static build, checks a pinned SHA256, and stores binaries and private
account state under `/data/hl-comma-remote`. It refuses a competing Tailscale
installation. The matching branch's `comma_remote` manager process recreates
the runtime systemd registration at boot, without writing the read-only OS.
Switching to a branch without this process removes that boot guarantee.

```sh
sudo python3 tools/jetlink/setup_comma_remote.py install --confirm-parked
sudo python3 tools/jetlink/setup_comma_remote.py login
sudo python3 tools/jetlink/setup_comma_remote.py status
```

Open the login URL as the device owner. Account enrollment/sharing cannot be
completed while comma is absent. No auth key or account state is committed.
DNS takeover, accepted subnet routes, exit-node use, route advertising and
Tailscale SSH are disabled. Ordinary comma SSH must already be enabled with the
owner's authorized key; sharing the Tailscale node does not install SSH keys or
grant root access. Restrict tailnet access to the intended operator and ports.

After normal SSH to the comma's Tailscale address, run `manage_jetson.py shell`.
Alternatively use OpenSSH ProxyJump from Mac with separate pinned host identities
and the Mac's already enrolled keys. No agent forwarding is required. SSH local
port forwarding can expose a selected Jetson service to the Mac without public
ports or subnet routing.

To stop remote access while retaining its identity:

```sh
sudo python3 tools/jetlink/setup_comma_remote.py disable
```

## Required Field Checks

1. Reboot Jetson and confirm the static USB profile and old authorized keys survive.
2. Connect the actual comma and check its management status, direct route and pinned SSH.
3. Unplug Ethernet and test USB SSH with Jetson Wi-Fi disconnected. Reconnect USB
   and verify recovery, SuperSpeed, real inference, and actual TURZX frames.
4. Re-enable the shared hotspot and prove LAN fallback after USB management loss.
5. Enroll comma in Tailscale, test from a different Internet connection, and repeat
   after reboot. Verify the operator's actual SSH access, not just Tailscale ping.
6. Run managed updates only parked with fresh offroad state. Automatic rollout
   remains disabled until the separate maintenance checklist passes.

On 2026-10-15 the actual C4/AGNOS 18.4 and carrot-jetson R2 pair passed USB SSH,
reboot persistence, the existing comma Tailscale SSH hop, managed install,
rollback/reinstall, unchanged-content skipping, and 1,200 synthetic inference
frames at 20 Hz (32.9 ms mean, 36.8 ms p95, one frame over 50 ms). Fresh HUD
snapshots arrived over the same USB link. Native Jetson GPU renders of dark/light
1920x480 fixtures matched local/transported state pixel-for-pixel.

TURZX was absent: real panel output, sustained display/inference concurrency,
vehicle camera inputs, live phone media, driving and a different-Internet VPN
path remain unverified. Automatic rollout and USB-only updates remain disabled.

### Carrot Web Verification (2026-10-15)

The actual C4/AGNOS 18.4 served both new panels on port 7000 after reboot.
The Jetson panel read Cinque v2, Orin-sm87, TensorRT 10.3.0, USB protocol 2,
and live health. All 20 one-second post-fix samples reported fresh health and
READY (52.156-52.281 C). Idle health replies now publish immediately rather than
waiting behind the one-second status writer. The original three-second freshness
threshold is unchanged.

The browser SSH check reached the pinned Jetson over USB. A theme change and
restore to auto succeeded through the browser; the resulting Param remained
comma-owned, mode 0600, and readable by the normal manager. Account-switch
confirmation/cancellation and invalid-key rejection were exercised. Existing
Tailscale account, addresses and SSH support-key list were unchanged; Tailscale
SSH to comma still worked after reboot. Manager reported no failed processes,
and panda/device/manager messages were alive and valid, offroad with no panda faults.

The 390-pixel layout had no horizontal overflow. The focused Jetlink, cluster,
UI badge, remote-support and sound-initialization suite passed 379 tests; Ruff
and changed JavaScript syntax checks passed. Fresh install, login URL extraction,
account switching, CSRF and onroad eligibility have automated test coverage;
an actual new-account OAuth enrollment/logout was intentionally not performed
on the owner's existing account. No physical TURZX or moving-vehicle validation
is claimed by these web checks.

### Desk Camera And Fault Validation (2026-10-16)

Real C4 road/wide/driver cameras, native QCOM model, driver monitoring and Jetson
Cinque v2 TensorRT inference ran in an isolated IPC namespace. Vehicle inputs in
that namespace were disabled/stationary fixtures. The real panda remained
`noOutput`, ignition off and `controlsAllowed=false`; no live controller was run.

After the USB-owner scheduling fix, a six-minute run contained 6,300 steady
model frames: mean 35.49 ms, p95 36.22 ms, maximum 38.85 ms, no frames over
50 ms, no invalid frames or frame-ID gaps. Concurrent driver monitoring also
had no gaps or invalid frames (maximum 31.89 ms). The initial Jetson join window
from 10-45 seconds had a 40.38 ms maximum. Cold native-model startup is separate
and can exceed 700 ms; these numbers do not claim a 50 ms bound during startup.
After the final manager reload, another 2,700 steady frames passed with a
37.81 ms maximum and zero gaps, invalid frames or frames over 50 ms.

Three deliberate three-second SIGSTOP stalls of the actual Jetson inference
service exercised the real USB timeout, not a locally closed socket. Each
recovered through the native model and rejoined Jetson. The failing frame took
182.63-188.09 ms and the model publication gap was 197.66-201.50 ms: the existing
150 ms IPC deadline plus native fallback does not fit a 50 ms frame budget.
Each incident produced 100 invalid model frames (about five seconds), and
camera odometry was also invalid throughout. Daemon/model socket and thread
counts were unchanged across the three cycles. Synthetic closed-socket fallback
timing is labelled separately in `bench_native.py`.

The existing steering-override join and three-second `commIssue` soft-disable
policy have not been replaced by immediate disengagement. Invalid model output
is not itself an immediate stop command. In-memory C4 state-machine tests cover
that existing timeout and no automatic re-engagement. Actual moving-vehicle
control, external-Internet access and physical TURZX output remain unverified.

### Connection Switch Verification (2026-10-15)

On the same C4, Tailscale stopped via the helper, rejected VPN SSH, and remained
Stopped after an offroad reboot. The saved account survived; cold-start status
now falls back to the selected saved profile when the daemon has no network map.
Browser tests covered reconnect, off confirmation/cancellation, confirmed off,
and reconnect again. The account, VPN addresses, SSH keys and all `tailscale get`
preferences were unchanged. Tailscale was left Running/online.

Jetson remained fresh READY while Tailscale was stopped, with pinned USB SSH
working. After reconnect, an actual SSH hop through the comma Tailscale address
also reached Jetson over USB. The 390-pixel screen had no horizontal overflow.
The expanded suite passed 403 tests plus Ruff and JavaScript syntax checks.

The pre-existing Wayon netfilter/iptables warning remains visible and its network
preferences were not altered by this feature. VPN SSH worked on the test LAN;
this does not establish a separate-Internet relay path. TURZX remained absent.

Official static installation documentation:
<https://tailscale.com/docs/install/linux#static-binaries>.

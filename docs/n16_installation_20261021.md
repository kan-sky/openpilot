# N16 Device Installation

## Scope

The user requested N16 on the paired Orin NX 8 GB and explicitly excluded the
external cluster. This installation changes only the inference runtime and its
comma adapter. It does not connect the experimental phone navigation producer,
alter vehicle control/Panda, replace the internal fallback model, or change HUD
assets. Without a qualified navigation producer the map and instruction inputs
remain zero, cleared each frame; this is not live route-following validation.

## Deployment Identity

The upstream Jetson image is boot-gated by a signed base-release manifest. Its
signing private key is not available here. A device-local, explicitly installed
inference overlay therefore retains the signed base identity and records its
own content identity separately. It is not an upstream-signed release.

The comma requires a matching `/data/models/carrot-jetlink-n16.json` selection
and the host's `carrot_n16_overlay` capability. Model, overlay content, base
release, runtime, protocol and stateful capability must match. A missing or
mismatched overlay on this selected Jetson does not silently select Cinque.
Other unselected Jetsons and supported phone/Mac apps retain their existing
selection paths. An upstream base replacement requires a new matching overlay.

The prepared TensorRT engine is specific to this NX/runtime. The experimental
engine and previous Cinque cache/release are retained for recovery. The N16
server refuses automatic in-process rebuilds. Empty-cache installation remains
unqualified and must not be described as generally reliable.

## Runtime Corrections

- First external inference failure now uses the same five-second invalid-output
  hold as an already active session; late local fallback is not marked valid.
- Raw prediction capture precedes destructive parsing for diagnostic parity.
- No timeout, frame validity, actuation or driver-monitoring check is relaxed.

## Verification

Before installation, 264 focused N16 and Mac-selection tests passed. A separate
fault/resource group passed 73 tests with 13 native-IPC cases unavailable on
macOS. These overlapping suites are not a unique combined count.

The combined camera/runtime/selection/fault/resource suite subsequently passed
483 tests, with 13 native-IPC cases skipped on macOS. A targeted protocol and
runtime rerun passed 255 tests with one native case skipped; these counts overlap.
Prior synthetic results are documented separately in
`n16_reverification_20261021.md`; they are not road validation.

## Installed Device

On October 21, the desk C4 and paired Orin NX 8 GB received the installation.
Fresh native evidence required offroad, harness `notConnected`, ignition off,
Panda `noOutput` and no driving processes. No force-onroad or vehicle control
message was used. The previous comma revision is retained as
`backup/pre-n16-20261021`; the signed Jetson base and previous engine remain.

- Model: `01effc679eb46d6036ce0302673a87fe2dcfcc95a679daeeb929679e53c3647a`.
- Overlay content: `3f757baca6566ccbf6b1f1b7ec52dafbc3c8ae54de83ba6ace91620b4d105960`.
- Base release: `84087a5b78118acc40234bfd8ef64235421f1aab`.
- Engine SHA256: `d13adb455630a200198cd24b89d646a4d960722643099e294c9e25f72662f84e`.
- Runtime: TensorRT 10.3.0, `Orin-sm87`, stateful protocol 2 over USB.

The native cached probe verified load, finite output and recurrent reset before
activating the root-owned versioned overlay. The comma reported exact
`N16 Nav (96244b1f)` and `jetSON READY`; this is not merely a model file on disk.

## Camera Test Findings

The isolated harness consumes actual road/wide camera buffers and driver-state
events, but publishes no driving output. Camera metadata uses native valid
CameraState events: this camerad does not populate `VisionIPC.valid`. Recorded
CalibrationParams supply an explicitly static warp, not fresh mounting/pose
validation. Camera and DM processes are stopped and the original CPU power
state is restored afterward by both normal cleanup and independent watchdogs.

The first complete 120-second run reached N16, injected a local IPC disconnect,
returned a finite internal output in 36.44 ms, kept the five-second invalid-output
hold, then rejoined N16. It produced 2,080 external outputs averaging 42.30 ms.
However, DM measured 19.8324 Hz with 22 missing frame IDs, so this run FAILED
the strict consecutive >=20 Hz requirement. Do not present it as a timing pass.

Investigation correlated almost every camera-drop event with the harness's
own periodic `/proc` scan: about 61 ms spent on the same FIFO54/core7 as model
execution. The driver monitor uses lower-priority FIFO5 on that core. This is
a measurement interference finding, not evidence to relax production timing or
DM requirements. The independent guard correction and repeat results follow.

The corrected guard runs full checks in a SCHED_OTHER/CPU0-3 watcher, outside
the realtime model core. The parent reads a bounded nonblocking mailbox; unsafe
samples latch and a missing/stale heartbeat fails closed. Its 7,971 parent checks
took at most 0.280 ms (p95 0.209 ms). The final camera/runtime/selection/fault/
resource regression group passed 594 tests, with 13 native-IPC cases skipped on
macOS. The 165-case camera harness rerun, including actual child-process mailbox
locking, also passed; these counts overlap.

The final 180-second measured run (`hl-n16-camera-v4.json`, private evidence)
completed with 3,499 finite, valid N16 external outputs: mean 42.725 ms, p95
46.805 ms, maximum 134.796 ms; 94 calls exceeded 50 ms. One camera frame was
skipped after N16 activation. Therefore this is NOT a strict consecutive 20 Hz
driving-model pass. Driver monitoring independently produced 3,600/3,600 valid,
consecutive frames at 20.0051 Hz over the measurement interval. This DM pass
must not be reported as a strict driving-model timing pass.

An injected local IPC close returned a finite internal output in 36.844 ms,
retained the five-second invalid-output hold, then rejoined N16. It did not test
physical USB unplugging. The unit exited successfully, restored CPU0-7, and
stopped its camera/DM children. A fresh native check afterward found offroad,
Panda noOutput, no unreadable Params, exact N16 READY over USB, and 3,509 host
frames served (includes warm-up). Static calibration, zero navigation inputs,
limited duration and no driving publications remain explicit limitations.

## Restart Findings

The overlay, engine and explicit selection survived a full Jetson restart.
The existing neutral USB-C boot policy initially negotiated the reverse role,
leaving the boot gate waiting for the comma link. A guarded invocation of the
already-installed policy helper restored the comma-facing role and N16 READY.
Persistent recovery must preserve PC USB access, signed updater behavior, and
offroad/inactive checks; a working manually recovered boot alone is not an
automatic-recovery validation.

The scoped recovery helper checks the explicit N16 pin, paired SSH host key,
direct NVIDIA `0955:7020` USB route, root-owned overlay contents and hash-pinned
installed policy helper. It requires 30 seconds of stable fresh offroad state,
inactive inference and the waiting boot gate. It performs at most one independent,
time-bounded role action per paired remote boot, and consumes ambiguous attempts
without redispatch. Fresh READY skips probes. Its separate diagnostic file cannot
replace updater results. The neutral PC boot policy and HUD updater are untouched.
The role/maintenance/USB regression group passed 256 tests before device deployment.

One comma restart also exposed a test-operator cleanup defect: the privileged
temporary operator unnecessarily rewrote `IsDriverViewEnabled=false`, leaving
that one Param root-owned mode 0600. The next manager boot could not copy it into
the boot log. A read-only scan found only this unreadable Param. Its false value
and mode were preserved while ownership was restored to comma; the operator now
asserts that the unchanged value remains unchanged instead of rewriting it.
A subsequent full comma restart reached fresh native offroad/manager/Panda state
with no unreadable Params. This was a test-induced permission failure, not a
model or low-voltage diagnosis, and is retained here rather than omitted.

The next full Jetson restart reached exact N16 READY automatically, with a new
boot ID, the same overlay, and zero inference-service restarts. That cycle
negotiated the correct USB role naturally; it did not dispatch the new recovery
helper. Therefore native automatic startup passed, while the helper's actual
wrong-role one-shot remains separately unverified on this final cycle.

During the boot-gate/runtime handoff one old release message (`0x4003`) reached
the inference server. Its existing health reporting showed a transient error
for 30 seconds, then returned to fresh READY without intervention. The native
engine remained loaded; do not misreport this as persistent N16 rejection or
claim the older handoff race was corrected in this patch.

## Carrot Web Model Identity

The tools card still said `jetSON · Cinque v2` even while native status reported
N16. The frontend hardcoded this title and `/api/egpu/model` returned only host
health, omitting the model. Use the same fresh `web_status()` snapshot as the
Jetson settings page, carrying its ready model/hash into the tools response.
Render the supplied name; missing or stale data shows a checking state, never
an assumed Cinque or N16. This is a display correction, not another model switch.
API regressions (45 cases), frontend model/card regressions (13 cases), and the
web bundle build pass. Regenerate the tools bundle/asset hash for cache busting.

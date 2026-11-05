# Traverse Manual Auto Hold

## Scope

Port only the disengaged, driver-pedal-armed stop hold from the maintained
Wayon reference `b7660d1d403938c464d4f14f842449e3f0e64ebd`. Do not port
engaged ACC hold/resume, automatic departure, or unrelated longitudinal tuning.
The previous speed-display and Navdy work remains separate and intact.

Only Traverse camera-long with the SDGM harness and no pedal interceptor is
eligible. New safety parameter bit 1024 accompanies the existing 515 bits,
giving 1539 when this interface builds eligible CarParams. Stock longitudinal
control and other GM platforms do not receive this feature.

## Behavior

- Arm on raw `ECMAcceleratorPos.BrakePedalPos >= 80` below 1.5 m/s. This is a
  raw CAN value, not 80 percent pedal travel. Arming while moving never brakes.
- Activate only with valid, fresh CAN, D/L, disengaged openpilot and stock ACC,
  no parking brake/gas/regen/fault, standstill and both speeds below 0.05 m/s.
- Retain after the initial pedal release. A subsequent light brake tap/release
  cancels; a strong re-press retains. Gas, motion, stale CAN, and lost eligibility
  clear the request. There is no automatic start or synthetic resume.
- Send only the existing bus-2 friction command, raw 400, mode 0xA. Suppress the
  existing automatic SET/RES path during manual hold; disabled gas stays inactive.

Panda independently requires the scoped flags, fresh wheel/brake/gas/cruise
inputs, driver arming, stationary state, no override or control permission, and
the existing address/bus/length/relay checks. The exception permits only positive
friction brake up to 400 in mode 0xA. It never grants `controls_allowed` or gas
permission. Required signal arrival order is tested in all 24 permutations.

Host cancellation observes gear, parking brake and a later light tap. Panda does
not independently mirror those three states: it retains a narrowly bounded
stationary-brake permission until its own motion/override/staleness reset. This
is a custom safety policy, not an upstream-certified feature.

## Verification

On the parked C4, in a separate staged checkout without CAN publication:

- 412 tests passed: manual hold, existing display conversion, and native C GM
  safety hooks. Includes real CAN parser/interface/controller and C hook calls,
  all 4096 brake encodings across buses, malformed/stale/missing inputs, packet
  ordering, initial-release retention, light-tap cancellation, gas prohibition,
  and unchanged ordinary ACC output.
- 148 existing cruise/button/state-machine tests passed with isolated Params.
  This includes the six IPC-dependent cases unavailable on the desktop.
- H7 application built with the target ARM toolchain and warnings-as-errors.
  Firmware source identity matches the local modified source: `118f42089854`.
- Signed application SHA256:
  `1e41201ab9a4be63db0758cd203381e6f6fcfd3d37e4a6c247fa4130def64459`.

## Device Installation

Target: `comma-db5ce68d`, WIP-FR at
`5378d124cb836b654611c6c4e21094b61e187774`, internal Cuatro/H7 Panda.
User confirmed Park, parking brake and ignition OFF. Fresh telemetry confirmed
offroad, both ignition flags false, noOutput, controls disallowed and no Panda
faults immediately before graceful manager shutdown.

Backup and staged evidence are retained on the device at
`/data/manual-hold-deploy-20261019-01/`. Only six runtime source files and the
H7 signed application were selectively installed. Existing device-only Traverse
mass 2090 and center-to-front ratio 0.7 were preserved in the deployed values
file; these were not imported into the local repository. Eleven other tracked
modified files were hash-verified unchanged. No branch/HEAD change or Git push.

The old overlay-init and finalized-consistency markers were moved into the
backup after the manager/updater stopped, preventing a pending old overlay from
replacing the selective installation on reboot. Web automatic pull was already
OFF; no persistent user update or driving setting was changed.

The application was flashed through the existing Panda API with heartbeat
checks retained. Bootloader/provisioning were not overwritten. An independent
read after flashing matched the entire expected 128-byte firmware signature,
and hardware reported no faults and no control permission.

The first deployment verifier stopped at its version-string assertion after a
successful flash. Existing `main_comms.h` omits the final byte of the generated
non-NUL version array: the device returns `DEV-118f42089854-DEBU`, not `DEBUG`.
Confirmed this pre-existing representation and rechecked the full signature,
all installed/preserved hashes, health and ignition before the requested reboot.
No safety or firmware source was changed to accommodate the diagnostic string.

Reboot completed with a new boot ID
`dde33454-598a-4f5c-b7f9-1be9dee34ecd`. SSH recovered on the same device;
WIP-FR and HEAD were unchanged. All seven installed hashes and eleven preserved
file hashes still matched. Fresh manager-start `PandaSignatures`, read by the
normal wrapper from the MCU, exactly matched the staged signature again.

A further 60.10-second live subscription verified fresh valid device/Panda/
manager messages, stable PIDs for UI, pandad, hardwared, Navdy bridge and
Carrot manager, no Panda faults or heartbeat loss, controls disallowed, noOutput
and ignition OFF. All CAN transmit counters and bus-error counters remained
zero. The SPI checksum counter was **1 throughout**, not zero: a startup
diagnostic was present, with no increase during this bounded observation.
Memory use settled from 44 to 38 percent; this is not a long-term resource test.

Using the installed Python sources and isolated Params, with the real enabled
AlphaLongitudinal/Openpilot switches, generated Traverse safety parameter 1539
and confirmed manual-hold support. The existing vehicle-specific mass and
geometry values remained intact. This used an empty fingerprint and did not
change the live safety mode or persist synthetic CarParams. Offroad's stored
CarParams may still contain the previous 515 until the next real onroad
fingerprint/configuration rebuild; `get_car` rebuilds it through `get_params`.

Private machine evidence is also copied to
`.analysis/manual-hold-deploy-20261019/` in the local checkout. The working-tree
implementation and this report remain uncommitted; this request applied the
device, not a Git publication.

## Limits

No live brake/gas command, force-onroad, moving test or physical hold test was
performed. Synthetic parser/controller tests and a correctly flashed MCU do not
prove ECU acceptance, available hold force, slope retention or release behavior.
These still require a supervised stationary vehicle check with the driver ready
to brake. Do not rely on this custom hold as a parking brake or assume its raw
400 command guarantees holding on every slope.

Before installation, the sleeping vehicle already had bus-2 form-error history
and error-passive state. This is not evidence that the new hold caused it, nor
proof that the bus is healthy with ignition ON. Reassess using fresh onroad CAN
if actuation remains unavailable; do not relax safety checks to hide it.

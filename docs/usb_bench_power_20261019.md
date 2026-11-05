# USB Bench Power Shutdown Correction

## Confirmed Source Defect

The migration's `PowerMonitoring.should_shutdown` ignored its existing `in_car`
argument. Consequently a voltage below 11.3 V requested shutdown even without
a vehicle harness. `calculate` also put standalone USB voltages into the car
battery low-pass filter and capacity integration.

The user first described no visible boot, then clarified that the Comma logo
appears before the device turns off. That is consistent with a post-boot power
shutdown, but it does not establish the actual device cause: no live voltage,
shutdown log, supply-current measurement or running-device access was available.
Insufficient charger/cable power can produce a similar observed symptom.

## Correction

- Pass the existing Panda harness-derived `in_car` value from `hardwared` to
  `PowerMonitoring.calculate`.
- Exclude non-car power from car-battery filtering and integration, preventing
  a low USB reading from contaminating the subsequent healthy-car calculation.
- Require `in_car` for the filtered-voltage shutdown condition.
- Preserve the 11.3 V threshold, ignition-on shutdown guard, configured offroad
  timer, and existing removal of boot/offroad grace and capacity-based cutoff.
- The offroad timer still applies outside the car. Saved settings are unchanged;
  support for 4320 minutes is not evidence of the current device's saved value.

The existing `DoShutdown` Param clears on manager startup. No persistent disable
of battery protection, voltage-threshold heuristic, firmware or bootloader change
was introduced. This correction does not supply power to an unpowered device.

## Validation and Limits

- Four new standalone-voltage regressions fail against the original HEAD policy
  (0, 5, 9 and 11.2 V filtered inputs) and pass after the correction.
- All 50 power-monitoring tests pass in an isolated desktop harness running the
  actual production policy. Native Params, hardware-power reads and log/stat
  endpoints were stubbed; this is not native Comma hardware validation.
- Tests cover healthy and low-battery car reconnection, bench exclusion from
  filtering/integration, harness removal, the exact threshold, timer boundaries,
  live timer setting changes and ignition-on shutdown prevention.
- Production call-site search finds one `calculate` caller, updated with the
  new required argument. All three changed Python files pass compilation.
- Policy and tests pass Ruff. The broader hardwared check retains the same
  pre-existing unused `UNREGISTERED_DONGLE_ID` import on HEAD and patched code.
- `git diff --check` passes. No device installation, reboot, Git commit or push
  was performed in this task; unrelated existing working-tree edits remain.

## Device Follow-Up

Power the Comma from a known-working supply/cable on its power input, or use
its normal vehicle harness while parked with ignition off. Verify device identity,
installed branch, harness status, raw/filtered voltage, timer setting and shutdown
logs. Deploy `hardwared.py` and `power_monitoring.py` together with the matching
checkout; do not replace only one file while hardwared is running. After an
authorized offroad restart, verify the USB bench case and vehicle battery guard.
Do not call the reported physical shutdown fixed until that check succeeds.

## Read-Only Device Follow-Up, 2026-10-20

Authenticated the same `comma-db5ce68d` at `192.168.35.175`, boot
`314b178b-186c-41a8-97de-41df1ba3220c`. The device remains WIP-FR at
`5378d124cb836b654611c6c4e21094b61e187774`. Both installed power files are
unmodified relative to that revision and still lack this correction:

- `power_monitoring.py` SHA256:
  `cd45c424a3675b2b7b844ffa0e496a97b120720f13a45f41334607067797de1c`.
- `hardwared.py` SHA256:
  `50154ee703919cfaf7a188ea2b87d7298721b496633631599646c52bd313f8ac`.
- The installed calculator still takes only voltage and ignition; the shutdown
  predicate still applies the 11.3 V condition without checking `in_car`.

Fresh valid telemetry showed offroad, harness normal, both ignition flags false,
Panda noOutput, controls disallowed and no Panda faults. Peripheral voltage was
11.704 V. The actual saved timer was 4320 minutes and DoShutdown was false.
Offroad carState was stale/invalid; its default gear and speed are not evidence
of Park or measured vehicle speed.

A separate Python process imported the installed production PowerMonitoring
class and called only its shutdown predicate. With ten seconds offroad,
ignition false and `in_car=False`, simulated filtered voltages of 0, 5, 9 and
11.2 V all returned true; 11.7 V returned false. The ignition-on guard and
healthy-car case returned false. The probe did not call calculate, write Params,
publish IPC, issue a shutdown, or alter the running hardwared process.
This confirms the installed policy defect, not the actual charger voltage.

No explicit device-shutdown event was found in the inspected recent retained
swaglogs. Broad text matches inside STATUS_PACKET were not shutdown events.
The journal retained only the current boot; the latest eight bootlogs did not
provide a matching shutdown decision. The current kernel reports an unknown
previous power-off reason, and its pstore is empty. Several bootlogs have stale
pre-synchronization wall times, so those dates cannot identify the home failure.
An abrupt power loss, supply limitation, or display-only failure is not ruled
out by missing shutdown logs.

No device source/configuration installation, network change, reboot, commit or
push was performed in this follow-up. The verified local correction still
requires deployment and a real USB-powered boot test.

## Authorized Device Installation, 2026-10-20

After the user requested applying the fix, rechecked the same device, branch,
HEAD and original file hashes. Fresh valid device/Panda/peripheral telemetry
confirmed offroad, both ignition flags false, controls disallowed, noOutput,
normal harness, no faults and 11.737 V immediately before manager shutdown.
After cleanup, a direct read of the same internal Panda confirmed ignition and
controls still false, no faults and 11.649 V. No firmware operation was performed.

Installed only these two runtime files as a matched pair, while manager and its
children were stopped:

- `power_monitoring.py` SHA256:
  `44b18ef0a4dafb7a6bba00def11be6a802c410040d7c00938d7336f68eb3259d`.
- `hardwared.py` SHA256:
  `9bdc79a571fb08c1b7842b274dfb1fd0a192fe59ad30c197e14ceba04ce1b8a6`.

Original files and deployment manifests are under
`/data/usb-bench-power-20261020-01/backup`. With the repository update lock held
and updater stopped, preserved the old `.overlay_init` and finalized
`.overlay_consistent` markers there to prevent a stale staged tree replacing
the selective deployment during reboot. Web automatic Git pull remained off.
Hashes of 22 unrelated modified/protected files, including the Panda firmware,
Navdy Bluetooth configuration and web settings, remained unchanged both before
reboot and during postboot verification. No Git commit, push, pull, branch
change, model/control edit or network configuration change was performed.

Thirteen isolated unittest cases passed on the device against the staged files,
then passed again against the installed files after reboot. They run the actual
policy code with in-memory Params and stubbed time, power reads and log/stat
endpoints; they do not write live Params, publish IPC or simulate real hardware
voltages. Cases cover bench exclusion, car reconnect/disconnect, 11.3 V boundary,
3-day timer boundary, live timer changes, ignition protection, power integration,
missing voltage/timestamp and the hardwared caller signature/syntax.

New boot ID: `2cac5055-2f2d-4240-ba07-71d63c35b9c4`. SSH recovered and installed
hashes, WIP-FR and HEAD `5378d124cb836b654611c6c4e21094b61e187774` were verified.
Across 65.04 seconds / 13 samples, hardwared PID 53313, pandad PID 53311 and UI
PID 53306 stayed unchanged. All required telemetry remained fresh and valid,
offroad, ignition false, controls disallowed, noOutput and fault-free; observed
peripheral voltage ranged 11.583-11.759 V. DoShutdown and DoReboot stayed false,
and the saved timer remained 4320 minutes. A bounded check of the current-boot
journal's last 700 lines found none of the selected power/argument/shutdown
errors; this is not an exhaustive log-cleanliness claim.

Private local evidence is under
`.analysis/usb-bench-power-deploy-20261020/device-evidence/`, including the
deployment event log, both isolated test results and post-reboot telemetry.

The software correction is now installed and running on vehicle power.
Actual boot from the reported home USB charger/cable and physical display output
have not been observed after this change. Supply insufficiency or another
independent fault is still possible; do not claim that physical symptom is
conclusively resolved until the USB-powered boot is checked.

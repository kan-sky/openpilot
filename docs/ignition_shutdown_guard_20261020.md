# Ignition Transition Shutdown Guard

## Request and diagnosis boundary

The user reported a Comma power cycle at engine start and authorized a
continuous-low-voltage check and a final ignition recheck. The preceding source
used filtered voltage below 11.3 V without a confirmation interval and latched
`DoShutdown`; the manager did not cancel that latch after ignition changed.
The observed physical restart has not been attributed to this path. Loss of
supply power, a platform reset, and application restart remain distinct causes.

## Policy

- Keep the 11.3 V threshold, configured offroad timeout (currently three days
  on the tested device), and USB-bench exclusion.
- Require raw AND filtered battery voltage below threshold continuously for
  ten seconds. This interval starts after the filter crosses the threshold.
- Reset confirmation on ignition, raw-voltage recovery, missing/invalid
  telemetry, harness removal, a measurement gap over 1.5 seconds, or a backward
  monotonic timestamp. Zero voltage is not accepted as a valid battery sample.
- Hardwared publishes a separate `AutomaticShutdownRequest` JSON parameter
  with reason and monotonic time, refreshed only while eligible. It no longer
  writes the user/manual `DoShutdown` latch.
- The manager requires fresh, valid offroad/device and known Panda state.
  Any Panda ignition signal or started device blocks automatic shutdown.
  Low-voltage shutdown additionally requires fresh low raw voltage and a
  connected harness.
- Drain subscribed messages and reread the request immediately before calling
  the existing hardware poweroff method. An expired request, recovered voltage,
  removed harness or newly received ignition cancels without stopping managed
  processes. Manual shutdown, uninstall and reboot retain their prior path.

For automatic shutdown only, request OS poweroff before manager cleanup, while
Panda and hardwared are still running and can supply the final check. Ordinary
manager cleanup follows. There is no irreversible `DoShutdown` latch waiting
through a long cleanup interval. This cannot cancel an OS poweroff already
accepted or account for an ignition change that has not yet reached the device.

## Verification and installation

118 focused tests pass: 60 power-policy tests and 58 automatic-shutdown tests.
The test harness stubs platform I/O and Params; no real poweroff, CAN publication
or vehicle actuation occurs. Checks include exact threshold/duration boundaries,
short crank-like dips, raw recovery before the filter recovers, missing and
reordered data, multiple Pandas, stale/future requests, real Cap'n Proto readers,
and execution of the actual manager loop with fake process/hardware endpoints.
The final-drain ignition case stays in the manager loop rather than tearing
down processes; manual shutdown and reboot remain separate.

The new parameter must be included in the normal native Params build. Deploy
hardwared, manager, both policy modules and the parameter registry together
through the normal build/restart flow; do not hot-copy only hardwared onto an
old Params binary.

## Home-charger regression investigation

The October 20 evening investigation found that the installed device still
ran the old unguarded voltage predicate. The noon selective USB-bench
deployment and its postboot verification had recorded the corrected source,
but the device reflog later recorded `reset: moving to HEAD` at 17:27:21.
The affected file's modification time matched that reset. Its current hash
matched the old tracked source and the subsequent vehicle-spec deployment's
preservation snapshot. Thus the recent vehicle-spec install did not introduce
the difference, but it also did not restore the already-lost power correction.
The reflog alone does not identify which person or process initiated the reset.

An isolated invocation of the installed old predicate returned shutdown with
ignition OFF, no connected harness, filtered voltage 11.0 V and only ten seconds
offroad. No actual poweroff or CAN message was issued by that reproduction.
Retained device logs contained no matching shutdown entry, so the exact
reported home-charger power cycle cannot be attributed solely from its logs.

The selective repair installs the three modified power/manager files, the new
automatic-shutdown module and the native registry key together. Existing
vehicle-spec sources and values, steering/braking code and the installed Panda
firmware are preserved. A source backup and native binary backup precede the
guarded manager stop and build. A physical 5 V charger boot and an actual
engine-cranking test remain separate from software-policy validation.

## Completed device validation

The guarded selective install and native build completed on October 20.
Twelve isolated checks on the actual device passed, including native JSON
Params roundtrips, USB/no-harness readings at 0/5/9/11.2 V, sustained vehicle
low voltage, raw-voltage recovery, the three-day timer and final-check
cancellation. The shutdown callable was fake and no CAN messages were sent.

Stopping the manager for installation caused the launcher to show its
`Manager failed to start` recovery display. The installer build and tests
had succeeded; the recovery updater refused both the active install lock and
uncommitted files, without resetting them. After the authorized reboot, the
recovery display process was absent and the normal managed UI was running.

Postboot observation lasted 90 seconds, with one stable UI, Panda and hardwared
PID throughout, fresh offroad telemetry, no ignition/controls, no Panda faults
and no automatic/manual shutdown request. All five installed source hashes,
17 preserved source/firmware hashes and the firmware-reported Panda signature
matched. The saved three-day limit and the existing 2090 kg / 0.70 vehicle-spec
settings were unchanged. The native isolated checks passed again after reboot.
This is not a physical home-charger boot test, a visible-panel confirmation,
an engine-crank test or driving validation.

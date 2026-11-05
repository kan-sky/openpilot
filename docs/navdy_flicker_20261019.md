# Navdy Logo Replay And Bluetooth Recovery

## Scope

The user reported repeated Chevrolet startup graphics and disappearing Comma
telemetry. The final requested test kept USB physically attached for Android
diagnostics while display snapshots and navigation feedback used Bluetooth.
This was a parked, ignition-on test with live Comma services, not an engaged
driving test. Fresh Park, zero speed, valid CAN, disabled/inactive controls and
no Panda controls permission were checked before interventions.

Only the Navdy APK and the reconnect ordering in the Navdy bridge changed in
this investigation. No Comma reboot, force-onroad, vehicle command, model,
controller, Panda firmware or phone/ambient bond change was performed.

## Confirmed Defects

### Unknown Transport State Replayed The Startup Graphic

The installed v160 receiver remembered the display payload's `onroad` boolean.
A transition from false to true called `TraverseBootView.replayIfAttached()`.
The Bluetooth expiry clearer intentionally supplies `onroad:false` with
`displayPower:"unknown"`. The next valid onroad snapshot therefore replayed
the 11.3-second Chevrolet animation and hid the Comma overlay, although the
device and app had not rebooted.

Before updating, the Navdy boot ID and the two app process IDs remained the
same as the earlier observation. Replaying the actual old smali transition
block with 20 expiry/recovery cycles produced 20 unwanted animations.

The v161 patch uses the existing strict `DisplayPowerHint.parse` result for
the ignition transition latch. Unknown, malformed, missing and bootstrap
states do not arm an ignition animation. Explicit offroad-to-onroad still
does. Display expiry itself is retained: old active information is not kept
on screen merely to hide a disconnect.

### Healthy Bluetooth Did Not Reset Reconnect Ordering

Selecting Bluetooth advanced the fallback candidate index. After a temporary
loss of an otherwise healthy wireless session, the next connection attempt
could therefore visit a paired Jetson relay first. The focused test reproduced
`bluetooth, jetson` before the fix.

After two seconds of successful Bluetooth exchanges, reset the candidate index
to the configured wireless peer. A failed Bluetooth reconnect still advances
to Jetson; USB remains the normal first choice. This adds four lines to the
already modified bridge and does not change lease, deadline or retry limits.

## Native Verification

- Installed v161 with `pm install -r`: `Success`. Read back the installed APK
  and verified its exact SHA256. App data and the peer-file hash were preserved.
- Captured actual Android framebuffers over the existing ADB server on port
  5038. The normal screen and screens 1.5/4.5 seconds after an unknown-to-onroad
  recovery were byte-identical. An explicit offroad-to-onroad transition still
  showed the Chevrolet graphic, providing the positive control.
- With production USB temporarily deauthorized, the bridge automatically used
  Bluetooth after 3.368 seconds. The remaining roughly 116.6 seconds retained
  one wireless session with zero expiry clears. USB was restored automatically.
  The background Android log file from this phase was empty, so this phase
  provides protocol/status evidence, not Android-log evidence.
- With USB still attached, a temporary bridge used only Bluetooth for actual
  live snapshots and feedback. The managed bridge was paused and always resumed
  in `finally`; feedback source timestamps and validity were not regenerated.
  USB was used only for logs and screenshots during this phase.
- Across that 180-second run: 886 successful exchanges, zero transmission
  errors, one session, zero receiver clears, zero source-dropout records.
  Receiver `usbUpdates` stayed exactly 596, proving USB did not carry the
  display snapshots. Median measured protocol RTT was 91.71 ms; maximum was
  284.25 ms. Maximum receiver-reported age was 206.14 ms, below the unchanged
  500 ms lease. The screenshot showed the normal Comma/music display.
- After installing the four-line reconnect fix, a separate 60-second test
  intentionally closed the Bluetooth socket once. There were 282 successful
  exchanges and exactly one injected failure, with no other transmission
  failures. The second Bluetooth connection was attempted after 2.042 seconds;
  the first applied-and-acknowledged fresh snapshot arrived after 2.825 seconds.
  Session changed 3 to 4, and the clear counter increased exactly once as
  expected. `usbUpdates` remained 1318. A screenshot 1.5 seconds after recovery
  showed normal telemetry, not the Chevrolet animation.
- Both test loops write a final `bridge_stopped` status deliberately; these
  terminal records are not unexpected wireless losses. Resuming the suspended
  production subscriber briefly recorded stale cached sources before draining
  fresh messages. There were no source-dropout records inside either Bluetooth
  measurement interval.

Only the managed Navdy bridge was restarted for its selective source update.
Model, driver-monitoring, control, card, selfdrived and Panda PIDs and the Comma
boot ID were unchanged. The diagnostic override was removed after testing;
the ordinary USB-first/Bluetooth-fallback policy was restored.

## Build And Regression Checks

- Five v161 patch tests, including old/new smali replay and preserved ignition
  transitions.
- Existing Java checks: 3263 wireless assertions, 70 display-power assertions
  and 72 wire-protocol assertions.
- 122 focused Python bridge/Bluetooth/transport/display tests pass after the
  reconnect correction, including healthy Bluetooth retry and failed-retry
  Jetson fallback.
- The broader pre-correction Navdy run had 175 passes and five ambient/DBC
  failures caused by missing local native `params_pyx`. It was not a full-suite
  pass and is not vehicle validation.
- Signature v1/v2/v3 and zip alignment pass. Only `classes2.dex` differs among
  non-signature APK entries; 7059 others and all 32 prior patch classes are
  unchanged. Existing class audit: 15697 verified, 27 prior added classes.
  DEX2 has 65533 method references, only three below its limit; future additions
  require attention to DEX placement.

## Installed Artifacts And Limits

Signed APK SHA256:
`4305ce2abd4d20404c86b5b9badd23754b5f4361507be45db013203957cdf87d`

Installed bridge SHA256:
`ec73453ec4df75e5ae4c53417ca5435e0ed16dca44dc0bb664522d83b9631210`

APK patch/build/tests are in the separate Navdy repository under
`hud_patches/v161-boot-transition/`. Private evidence and rollback copies are
in its ignored `output/navdy-boot-transition-20261019/` and the Comma's
`/data/hl-navdy-boot-transition-20261019/`. Preserve the v160 APK, signer and
pre-change bridge. Do not uninstall or clear Navdy application data to roll back.

The original receiver counters show earlier clear events, but these tests do
not establish the physical cause of every historical disconnect. HERE-service
DNS retries and Android log load were also observed; they are not proven causes.
No spontaneous failure occurred in the bounded Bluetooth tests. That does not
prove long-drive stability, active geometry throughput, simultaneous phone and
ambient behavior under every load, optical HUD output, or navigation-controlled
vehicle behavior. Framebuffer captures and application ACKs are the verification
boundary here. No Git publication was performed by this investigation.

## Follow-Up: Leave Bluetooth Active With USB Attached

The user subsequently requested leaving the actual data connection on Bluetooth,
not restoring USB-first after the diagnostic interval. Add an explicit boolean
`bluetooth_only` to the existing private paired-peer configuration. Its default
is false, preserving the earlier automatic fallback behavior on other devices.
When true, the managed bridge uses only the paired Bluetooth transport, including
after errors/restarts. It neither probes/writes direct USB nor tries Jetson.
USB ADB diagnostics remain available. Runtime status reports `transportPolicy`.

Read the peer and policy atomically from the same bounded configuration snapshot
every five seconds. Policy/peer changes close the old writer before connecting
the replacement. A disabled peer in Bluetooth-only mode remains disconnected;
it does not silently send data over USB. A corrupt live configuration retains
the selected policy, disconnects the old peer and reports the parsing error.
Removing the configuration intentionally restores the default USB-first policy.

All137 focused bridge/Bluetooth/transport/display tests pass. New cases cover
explicit policy parsing, USB-attached Bluetooth-only operation, wireless retry,
missing/disabled peer, corrupt live configuration, and live changes in both
directions. The native install touched only the bridge and Bluetooth helper,
preserved the peer/bond and restarted only the Navdy bridge after fresh Park/
zero-speed/inactive checks. Comma boot and model/DM/control/Panda PIDs stayed
unchanged. No additional Navdy APK installation was needed.

Installed follow-up bridge SHA256:
`f2e8403b3a866d7fbb51b033264c4cc36d97271146f2e582304d12d8cdd87f85`

Installed Bluetooth helper SHA256:
`6ed21ff836a34567630a9a848097d64479eb432fc3569752d636bc14c278d312`

The private `bt-only` subdirectory of the existing evidence archive contains the
deployment guard/hash report, pre-change files/configuration, USB logcat output,
sampled sender/receiver diagnostics and actual framebuffer captures. Unlike the
earlier paused-bridge experiment, this uses the ordinary managed bridge throughout
and leaves its Bluetooth-only setting enabled after collection.

The new USB-log interval ended after126.44 seconds because the user physically
unplugged USB, subsequently confirmed by the user. Kernel monotonic3309.671
records `usb 3-1: USB disconnect`, followed by Type-C detach and host-controller
removal. Do not classify that logger exit as an unexplained USB failure or claim
the planned180-second collection completed. The saved1234 samples contain614
distinct matched sender/receiver acknowledgements: all connected Bluetooth-only,
session5 throughout, clear counter unchanged at1, USB update counter unchanged
at4270, no source-dropout records. Median sampled RTT94.625ms, maximum322.45ms;
maximum receiver age267.12ms. The captured framebuffer shows normal telemetry.
The27 filtered Android log lines show fresh/awake power state and no fatal app
exception; they are not an exhaustive Android log.

A subsequent read observed session6 and clear counter2, proving one additional
wireless session/display-clear transition outside the captured USB interval. It had
already recovered with fresh acknowledgements; exact interruption timing and
cause were not captured. At that read Comma reported active control, so no
further install, process restart, forced disconnection or fault injection was
performed. The following observation is passive status-file sampling only.
The kernel also retains the previously seen unknown-ACL-handle messages; their
relationship to this particular reconnect is unproven. Do not report this entire
follow-up as zero wireless failures or complete long-drive stability.

The final60-second passive interval has591 samples and299 distinct reported
wireless sequences, all fresh/connected on session6 with clear counter unchanged
at2 and no source-dropout record. Median sampled RTT89.55ms, maximum257.64ms,
maximum receiver age203.3ms. The last fresh query confirms Bluetooth-only still
enabled persistently, USB absent as requested by the user's physical removal,
and the same managed bridge PID110483. Both installed module hashes match the
follow-up above. Diagnostic collectors are stopped; the managed Bluetooth data
connection remains running, including if USB is subsequently reattached.

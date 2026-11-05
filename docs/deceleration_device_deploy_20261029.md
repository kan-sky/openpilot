# Deceleration Display Device Deployment

## Installed Versions

- Comma production code: `a7d35461eab8300b7c7d5ed0ced6ba2acb60607d`,
  published to `leehyuk1108/carrotpilot` / `WIP-FR` and installed by fast-forward.
- Navdy v173: existing-signature `pm install -r` succeeded. Installed APK SHA256:
  `fc0460b05dc2ef7fa4c54da56babd12a70b81360b0bc09246cbb7c0230b1b22f`.
- Private Navdy source archive: `leehyuk1108/navdy-phone-source` / `main`,
  `ceebdfb3703c171bb747e86d7de5473b273189eb`. All 1,001 manifest-listed source
  files were hash-checked; APKs, signing material, pairing data and raw device
  logs were excluded. This archive is distinct from the mixed local history.

Only the requested display/schema/sender files were deployed. Existing dirty
DBC files, unrelated auto-update work, model choices and settings were retained.
Vehicle-control calculations, Bluetooth cadence, phone reconnection and ambient
logic were not changed. NewUi remains enabled. Navdy placement is the former
auxiliary-speed number anchor: x390, y297, width48, height15. See the paired
[display contract and renderer checks](navdy_deceleration_phases_20261029.md).

## Actual Device Checks

Deployment and restart required fresh offroad, ignition-OFF and controls-inactive
observations. The Comma then booted with the new commit, completed its native
schema/dependency build and started the normal manager. Its previous staged
overlay was explicitly skipped; it did not replace the installed revision.

The postboot 65-second observation confirmed:

- Stable live UI, carrot_man and navdy_bridge processes.
- 1,300 actual carrotMan updates with the new candidate field readable.
- All 15 selected non-document source hashes match the installed commit.
- Existing tracked local DBC changes and selected settings match pre-update hashes.
- All 64 one-second Bluetooth samples connected under `bluetooth_only`, with
  fresh status, advancing sequence 427 to 744 and feedback arriving normally.

Focused tests on the installed Comma tree passed: **380 passed in 11.98 seconds**
(candidate producer, Navdy verdict/display and C4 New UI). Two pytest warnings
refer to unavailable C++ plugin configuration, not test failures. No synthetic
engagement, control or CAN messages were published to the running system.

On actual Navdy API22/ART2.1.0, a separate process loaded the installed APK's
badge class and passed **33,569 assertions**, all 42 source/phase codes and 47
native PNG outputs. The 14x3 atlas was visually inspected. No live receiver,
Bluetooth owner or window was invoked by that test. This is installed native
software rendering, not optical HUD or engaged-driving validation.

After the APK update, a separate 180-second observation passed with stable app
and connection-service PIDs, Bluetooth ownership, zero USB display updates and
unchanged pairing/registered-phone/policy fingerprints. The first observation
had stopped on stale status during service re-advertisement; the install itself
had succeeded. Normal automatic reconnect recovered without reinstallation or
pairing reset. Both observations are retained, including the unsuccessful one.

After the Comma reboot, a further 60-second / 80-sample Navdy observation passed:
the same app/service PIDs survived, Bluetooth remained the owner, USB display
updates stayed zero and policy fingerprints matched. The final live status had
no last failure. The Android crash buffer was empty at collection; that alone
is not proof that no earlier crash ever occurred.

## Limits

The wider native source-overlay suite passed 1,466 tests with one inherited
in-place-file-overwrite timestamp test failure. Unchanged baseline code also
reproduced it (26/30 attempts versus 28/30 overlay). Production atomic replacement
and its reader were not modified. This is not a whole-suite pass.

Actual optical readability, a live engaged display and long-drive radio stability
remain unverified. Green means the reason's speed cap reached its endpoint, not
that the vehicle physically finished braking. SECTION is fitted into a narrow
slot; OpReady touches the badge's top boundary without area overlap. No physical
clearance or readability claim is made from the isolated pixel test.

Private deployment records and recovery APK remain on the devices and in ignored
local build/evidence directories. They are intentionally not included in Git.

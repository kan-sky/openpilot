# Independent Navdy section enforcement

## Observed failure

The November 1 recording on baseline `420bdc03` shows a section limit of
110 km/h and approximately 7.9 km remaining replaced by a mobile-camera
observation. Mode 2 correctly excluded that mobile camera, but the shared
camera slot had already lost the section. The navigation cap disappeared for
37.654 seconds before section data returned. Bluetooth acknowledgements
continued during that interval. This establishes the receiver-side loss, not
the exact upstream SDK candidate or actual overspeed. The VTURN display patch
did not change these input/selection files.

## Contract

- The phone captures section state only from an actual navigation RG callback,
  using `getSpeed().getSection()`. Other callbacks must not refresh its age.
- A separate versioned section object carries a phone-process session, source
  sequence, elapsed age, active/suspended/off-route flags, limit and distance.
  Queued frames must be aged again at the actual transport write.
- The existing HUD JSON relay carries this object without a HUD APK change.
- Comma keeps point cameras and section observations independently. Existing
  mode 2 mobile-camera filtering no longer removes a valid section constraint.
  A lower fixed-camera cap still takes precedence; the section remains after
  that point camera is passed. Equal caps prefer the section display.
- Each channel has independent ego-motion distance bookkeeping. Changes in
  the other channel or repeated source values cannot reset its travelled
  distance, including frozen 8 m and 26 m observations.
- Section source lifetime is at most four seconds, additionally bounded by
  the existing HUD deadline and two-second link freshness. Duplicate packets
  cannot extend that lifetime. Clearing preserves the generation watermark.
- Explicit stop, suspension, off-route, invalid/null data, expiry and consumed
  distance remove the section. The patch never latches an obsolete speed limit.
- An upgraded source owns section validity, including explicit clears. Its
  legacy section-camera fallback is suppressed for that HUD session. Old
  phones without the independent section object retain their existing path.
- Network navigation, mode 0, steering, Panda, driver monitoring, Bluetooth
  policy and automatic phone reconnection remain unchanged.

## Verification

The focused receiver/parser/actual publisher suite passed 185 tests on the
desktop and in an isolated candidate namespace on Comma. Inputs were synthetic;
no live CAN, engagement or actuator commands were injected. Coverage includes
100/110 km/h, modes 0-3, camera replacement, tighter fixed-camera limits,
independent distances, source/link expiry, lifecycle clears, HUD restarts,
replayed generations, legacy compatibility and frozen near-end distances.
The checked-in JSONL fixture is synthetic SDK data emitted by the actual Java
SectionRelay helper (release-r3 host probe), not a hand-invented wire schema.
Those exact integer fields reach section/camera caps and an explicit clear.

The broader native Navdy suite passed 1,313 tests, with one existing file-reader
test failure also reproduced on the unchanged installed baseline. The failing
test overwrites the same inode immediately and the reader uses filesystem
mtime/inode change detection; production feedback uses atomic replacement.
A /data temporary-directory run additionally failed a related model-cache
test. These remain reported failures, not a full-suite pass. Initial 36 journal
failures were caused by omitting the suite's isolated-journal fixture with
`--noconftest`; restoring that fixture resolved all 36. Those initial runs
did not have the intended journal-path isolation and are not counted as passing
validation. No journal storage policy was weakened.

Claude's final read-only Comma review found no blocking defect in the frozen
candidate. Device tests and the build are performed separately from that review.

Phone build, cross-language fixtures, final review and deployment results are
recorded below when completed. Source/static tests are not road validation.

## Remaining boundaries

This repair requires both the updated phone producer and Comma consumer. With
the old phone alone, the independent section evidence does not exist. A real
drive through consecutive hazards, actual SDK update cadence, radio delays and
physical deceleration remain separate validation steps. Loss of fresh section
evidence still clears that constraint; do not rely on this assistance as a
guarantee of section-average speed compliance.

When reverting to an old phone app, restart the HUD relay to begin a new HUD
session. Otherwise its sticky independent-section capability deliberately
continues suppressing the obsolete legacy section slot. Bluetooth reconnection
alone does not grant old evidence a new source generation.

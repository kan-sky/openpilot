# C4 deceleration badge phases

VTURN/ROUTE preparation and unused-completion behavior below is superseded by
`curve_applied_only_20261030.md`. Other reasons retain the original phases.

## Scope

Source changes on the WIP-FR publishing checkout, based on
`1e47d6df579f0f168aa814ff524949d8d14d2b5e`. No device installation or Git push
was performed in this implementation turn. Existing unrelated update changes
are not part of this patch.

Only the New UI C4 badge changes presentation. Its size, position, labels,
fonts, warning/timer priority and the other HUD elements are retained.
Steering, braking, acceleration limits, speed calculations and settings are
unchanged.

## Display Contract

- Disengaged, longitudinal control inactive, stale/missing input or no valid
  candidate: hidden. Existing critical warnings and timers also hide it.
- White: valid information, waiting for its cap to apply. Driver override or
  an unmatched planner target also leaves information white, never green.
- Orange `#FF9500` alternating with white at 1 Hz: this reason owns the applied
  speed cap, but its final target has not been reached.
- Steady green: that reason's published integer cap reaches its own endpoint,
  or the driver's selected speed is already at/below that endpoint.
- Actual vehicle speed is not a completion input. Green does not certify
  physical braking completion or legal speed compliance.
- All currently produced reasons participate: CAM, SECTION, BUMP, POLICE,
  WAZE, TURN, SCHOOL, ROAD, ROUTE and VTURN. The retired MODEL producer is not
  reintroduced merely because a legacy label still exists.

CarrotMan's additive `decelerationCandidates` field contains each enabled
reason's unrounded cap, endpoint and distance when known. UI does not duplicate
the deceleration equations. Route endpoint ownership is tracked alongside the
existing reverse speed pass; vision endpoint metadata is accepted only when
it matches the current filtered cap. A held cap with an unknown endpoint may
blink but cannot invent completion. Curve green needs 0.3 s of continuous
completion; that confirmation interval is white, not orange.

One badge remains. Priority is an actually applied reason, then a pending
required reduction, then an unnecessary reduction. Lowest cap wins within a
priority group; known distance breaks ties. This prevents an unnecessary green
curve from masking a pending camera and a distant turn from winning a tied
envelope over a nearer camera. Valid ROAD/curve information can remain visible
in green while engaged, as requested. No arbitrary distance cutoff was added.

## Verification

- 144 C4 UI tests pass, including all reason labels/phases, real-speed
  independence, disengagement, stale data, overrides, neutral caps, multiple
  reasons, consecutive cameras, green confirmation and existing HUD layout.
- 272 producer/schema/curve/navigation/baseline tests pass. This includes
  80 settings combinations with 40 inputs each: all pre-existing published
  CarrotMan/navigation fields match the baseline across 3,200 synthetic steps.
- Real Cap'n Proto serialization/deserialization passes with the added field;
  a message without candidates remains hidden instead of guessing completion.
- Ruff on the display helper/tests and `git diff --check` pass.
- Claude independently reviewed the pinned diff. Its pending-event priority,
  distance tie and false-orange confirmation findings were fixed with tests.

Desktop execution used Python 3.12 and real schemas/calculation code. Missing
native Params/msgq/hardware I/O was replaced with test-only stubs; UI drawing
was captured through the existing raylib test fixture. This is not a native C4
build, rendered-device capture, live sensor test or road validation. Local
review snapshots and the desktop equivalence runner are retained under
`.analysis/c4-decel-20261029/` (not published).

Deployment must include the schema, producer and UI together and verify their
versions after restart. Older publishers provide no candidate metadata and
intentionally do not trigger the new three-phase badge.

# Parking capture diagnostics, 2026-11-04

## Scope

Add bounded counters to the existing parking-approach capture path. Preserve
camera selection, JPEG quality 92 / 4:4:4, native dimensions, capture/thermal/
memory gates, sampling and backoff, buffer retention, upload freshness and
frame selection. No model, steering, braking, manager or camera policy changes.

The goal is to distinguish missing approach photos caused by eligibility gates,
IPC delivery, encoding/backoff, buffer clearing or final upload filtering.
Counters do not fix or relax any of those conditions.

## Storage

- Worker checkpoint: existing `/dev/shm` capture directory, `diagnostics.json`.
- Latest final summary: `/dev/shm/wayon-parking-diagnostics.json`.
- Fixed counter allowlist, saturating integers, maximum 8 KiB JSON, worker
  checkpoint at most once per five seconds plus terminal flush.
- No image data, IDs, location, timestamps, arbitrary text or exception contents.
  No network upload or persistent append log. Atomic replace with no-follow and
  regular-file checks; diagnostics failure cannot block the capture path.
- A forced kill can lose counts since the previous checkpoint. Missing and
  unfinished worker snapshots are explicit; counters are not lossless tracing.

`ApproachFrames` remains a list-compatible collection with bounded counter
metadata passed to the existing uploader. No wire format change is introduced.

## Verification

- Local focused suite: 113 pass (50 existing plus 63 diagnostic tests).
- Actual Comma Python runtime, isolated candidate imports and scratch cleanup:
  113 pass. These are runtime tests, not real camera/IPC capture validation.
- Actual recorded inputs: 18 qlogs across 5 drives, 9,114 device/car samples;
  old versus new capture-eligibility predicate changed on zero samples.
- Focused Ruff and whitespace checks pass.

## Installation

Target only the user's Traverse `WIP-FR` branch. Preserve unrelated generated
DBC modifications, settings and untracked build/model artifacts. Require fresh
offroad device state, baseline commit and the exact tested files before a
single-parent fast-forward. Back up the old parking module, then restart only
the managed `wayon_cloud` uploader to import the new Python module. Do not
force onroad or reboot the system for this diagnostic change.

Actual next-drive capture counts, physical parking photos and sustained resource
cost require future observations. The server multi-stage comparison is separate
and does not replace current parking recognition notifications.

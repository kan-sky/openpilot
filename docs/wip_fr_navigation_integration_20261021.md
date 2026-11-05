# WIP-FR Navigation Integration

## Scope

Integrate the six reviewed carrot-nav changes into WIP-FR, based on published
`52782e1397efedabfd6db7a17c34371d098a5b2f` and inspected upstream
`59eb0f82fa8bf5f2d85106afa446d33197c52df8`. This is a scoped backport, not a
replacement of the custom branch. The original dirty development worktree is
not the publication source.

- N16: connect CPU map rendering, selected-route identity, original instruction
  observations and optional inputs to the existing pinned Jetlink adapter.
- DM: backport animated eyes without replacing Wayon phone-risk warnings or
  monitoring/control policy. The separate mici driver-state implementation stays.
- Jetson: distinguish bounded initial HELLO/model preparation wait from errors;
  retain detailed failures, guarded joining and native fallback.
- Memory: evict stale/reused-PID proclog cache entries and avoid the unused
  observer when the internal driving model is only an external-GPU standby.
- GPS/navigation: original fix timestamps, bounded source selection, Park-only
  stationary anchor and asynchronous bounded Mapbox requests.
- Vision BSD: independent default-OFF `OnnxBsdEnabled` setting; unload disabled
  networks and reject in-flight results across setting generations. Lane and
  OEM blindspot paths are retained. Existing general TCP telemetry is retained.

## Navigation Contract

`navRoute` and `navInstructionCarrot` carry a matching source and nonzero route
generation. A route heartbeat does not renew an original guidance observation.
Legacy network, v2 network, Mapbox and Navdy are not interchangeable. Each v2
current/next instruction retains its own source, session, timestamp and distance.
Invalid modifiers, unknown current turns, stale/future observations and old
instructions from a previous route fail closed.

Only the exact active N16 Jetlink contract consumes these inputs. Missing input
is zeroed on every frame; internal/Cinque/eGPU paths are unchanged. Route receipt
must be less than 3 seconds old, guidance/projection less than 2 seconds old,
and the map less than 500 ms old from render start. Slow tile downloads cannot
refresh an old route or make an old image appear newly captured.

The existing VisionIPC map publication remains for other consumers. Modeld
does not connect to it: native `connect(False)` can still wait indefinitely for
buffer FDs. Instead mapsd atomically replaces one fixed-size 65,560-byte tmpfs
snapshot containing magic/version, route generation, monotonic capture time and
256 x 256 grayscale bytes. Modeld performs a single bounded nonblocking regular
file read, with no socket handshake, network request or renderer lock. FIFOs,
symlinks, oversized/truncated data and wrong versions are rejected. `mapsd` is
an optional process, not a new controls-readiness prerequisite.

## Existing Custom Behavior

No GM controller/safety, NNFF selection, manual auto-hold, vehicle-spec tuning,
power-off policy, Navdy Bluetooth/USB transport, OBD speed or ambient lifecycle
implementation is replaced. Navdy guidance only gains original observation
metadata. Mapbox credentials and the configured navigation source are preserved;
the web does not silently switch away from Navdy.

## Important Limits

The subsequent model-navigation transport work is documented in
`navdy_model_navigation_20261021.md`. The following describes this integration's
original baseline before that separate phone/HUD/comma extension.

Current Navdy feedback supplies turn guidance, not the complete route geometry.
Selecting Navdy therefore still leaves the coupled N16 map/navigation input
unavailable until that transport supplies a full route. Existing Navdy display,
speed-limit/camera and TBT handling continue independently. Do not claim full
phone-to-N16 navigation merely because N16 inference is running.

CPU rendering and optional-input tests are not camera/DM/Jetson sustained-20Hz
or closed-loop road validation. No forced onroad, Panda firmware update, model
weight replacement or vehicle-control test is part of this integration. The
earlier Jetson cached-engine measurements remain historical, separate evidence.

## Verification

- Desktop Jetlink regression: 1,534 passed, 19 skipped. Linux-only HUD/storage
  groups and the native raylib health test are not part of that run.
- Desktop integrated navigation, settings, DM, process-cache and model tests:
  574 passed, 11 skipped. Native rasterizer coverage is supplied below.
- Comma isolated native candidate, with fresh offroad/noOutput guards: 1,160
  passed and 146 subtests passed in 66.40 seconds. This includes the actually
  compiled Cython rasterizer, map/provenance/N16 adapter, Jetlink fault isolation,
  Linux HUD/storage, Navdy, GM manual hold/display speed, NNFF substitution,
  vehicle settings and both HUD status paths. Tests use a separate Params prefix;
  they do not replace live processes or exercise vehicle actuators.
- Snapshot/adapter tests include actual writer-to-consumer pixels, atomic
  replacement, bounded reads and a subprocess deadline for a FIFO path.
- Web navigation/BSD/vehicle-spec contracts: 45 JavaScript tests passed.
- All 29 generated web-asset hashes were checked after the combined rebuild.
- The complete wiki-settings catalog still has two baseline failures for the
  custom VehicleMass/VehicleCenterToFrontRatio defaults. The same failures exist
  at the parent commit; the scoped vision settings check passes.

Publication is followed by a separately guarded device fast-forward and reboot.
The test results above do not claim that deployment or a road test has occurred.

Counts describe separate, potentially overlapping suites, not a unique total.

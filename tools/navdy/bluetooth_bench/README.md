# Navdy Bluetooth Bench

This is an offroad, synthetic radio benchmark, NOT a production Bluetooth
transport. It never sends telemetry to the HUD, navigation to openpilot, CAN,
ambient commands, images, audio or video. Existing USB traffic can remain active.
Do not infer driving readiness or phone-navigation concurrency from its results.

The follow-up snapshot experiment uses a separate UUID
`a8de4aa2-5600-47cb-8b2d-3a7dabd3a918` and `BenchMain serve-snapshot`.
It has a synthetic receiver state only: it never invokes the production HUD.

## Boundaries

- Separate secure RFCOMM UUID: `b910e2b5-1b15-4b3d-88a0-4d803ed0f7ce`.
- Exact previously paired comma MAC allowlist; encrypted/authenticated socket.
- No adapter resets, unpairing, auto-pair approval, startup registration or OS flash.
- Maximum 32 KiB per frame, monotonic sequence, CRC32 and full deterministic
  payload comparison in BOTH directions. Stream fragmentation is supported.
- One outstanding request, matching the current bridge's request/reply pattern.
- Server exits after ten minutes or a ten-second stalled exchange. Client bounds
  every exchange to two seconds, and checks `IsOnroad=0`, `IsEngaged=0` each time.
- The native runner is only for an existing authorized root ADB shell. It does
  not obtain root, change installer policy or modify the production HUD APK.
- Pairing keys and test signing key stay outside Git. Do not publish device dumps.

## Build

Use the existing JDK and Android SDK; no Gradle or SDK installation is required.
`build.py` requires Build Tools 35.0.0 and platform android-35. Output directories
must be new. Generated DEX targets API 19 and was exercised on Navdy API 22.

```sh
python3 tools/navdy/bluetooth_bench/build.py --jdk "$JAVA_HOME" \
  --sdk "$ANDROID_HOME" --out /absolute/new/private/output
python3 -m unittest discover -s tools/navdy/bluetooth_bench -p 'test_*.py'
javac -d /tmp/hl-bt-test tools/navdy/bluetooth_bench/android/Protocol.java \
  tools/navdy/bluetooth_bench/ProtocolTest.java
java -cp /tmp/hl-bt-test com.hl.navdy.btbench.ProtocolTest
```

The optional APK has a separate package and signer. The tested Navdy firmware
rejected its installation (`Installation of new packages is disabled in user
builds`). Do not disable verification. Instead, use the temporary DEX runner
through the device's already-root ADB session. No APK was installed in this trial.

## Native Trial

Inspect the exact serial, installed HUD package, radio state and current offroad
state first. Never use an unqualified `adb` command when multiple devices exist.
The production comma bridge uses an isolated ADB server on port 5038.

```sh
adb -P 5038 -s "$NAVDY_SERIAL" push classes.dex /data/local/tmp/hl-bt-bench.dex
adb -P 5038 -s "$NAVDY_SERIAL" shell \
  'CLASSPATH=/data/local/tmp/hl-bt-bench.dex app_process /system/bin com.hl.navdy.btbench.BenchMain status'
```

Pair only the physically identified Navdy and comma, with user authorization.
The ordinary Android discoverability confirmation can be opened with
`android.bluetooth.adapter.action.REQUEST_DISCOVERABLE`, duration 120 seconds.
Do not force pairing confirmations or alter other bonded devices.

Run `BenchMain serve COMMA_MAC` in place of `BenchMain status`. Keep the process
attached to a bounded ADB session. Read its `READY` line, then discover the fresh
RFCOMM channel for **this UUID** with `sdptool browse NAVDY_MAC`. Channel numbers
are dynamic; channel 3 in the recorded trial is not a permanent contract.

Copy `client.py` to comma `/tmp/client.py`. From comma:

```sh
python3 /tmp/client.py --peer "$NAVDY_MAC" --channel "$CHANNEL" \
  --seconds 60 --hz 5 --tx 2409 --rx 1932
```

`--hz 0 --tx 8192 --rx 8192` measures bounded stop-and-wait bulk throughput,
not theoretical radio capacity or fully pipelined throughput. Sizes INCLUDE
the 20-byte benchmark header. Lower-size trials simulate a smaller payload;
they do NOT implement or verify production JSON compression.

`campaign.py` retains sequential trials as private JSON files. It assumes the
verified client is already copied, the paired secure server is running, and
the current channel was checked. It stops on an error and never enables radios.

## Latest Snapshot Experiment

`make_fixture.py` (Python 3.10+) generates a synthetic full-state JSON using the
existing display serializer, including eight geometry lines, two leads and UTF-8.
`snapshot.py` generates changing copies at 5 Hz, with one unsent slot. A newer
snapshot replaces only that slot; it cannot cut an in-flight stream frame.
Commands and transient warning events are NOT routed through this slot.

Both raw JSON and zlib level 1 are tested losslessly. Decompression bounds,
CRC32, complete byte comparisons and sequence/lease checks apply at both ends.
The response echoes the decoded-and-validated frame, not real navigation data.
Bidirectional sizes are consequently equal rather than a measured TBT workload.

Receiver freshness uses its last ACK/hello as a lease: the sender reports capture
time relative to receiving that lease, not an absolute wall-clock timestamp.
The receiver computes `elapsed_since_lease - capture_delta`, a conservative age
that also includes the prior ACK transit. Add 2 ms plus 1000 ppm elapsed-clock
margin. This assumes honest monotonic timestamps and bounded clock-rate error;
it is not a general clock-synchronization protocol or production safety proof.
The experimental TTL is 500 ms. Expired/reordered states cannot refresh the
previous state's remaining lifetime. A separate 100-ms watcher clears expired
stored state; synchronous reads also check expiry. No pixel rendering is tested.

Run `SnapshotProtocolTest.java` with `Protocol.java` and `SnapshotProtocol.java`
using the same standalone javac procedure. `test_snapshot.py` covers bounded
coalescing, immutable in-flight selection, UTF-8 and malformed compressed input.

After pushing the verified DEX, `snapshot.py`, `client.py` and the generated
fixture, check the new UUID's SDP channel. If Navdy was non-connectable, the
helper `trial-connectable EXACT_PAIRED_COMMA_MAC` temporarily makes it connectable
without discovery or adapter reset. Record and restore the original scan mode.

`snapshot_campaign.py` runs raw/compressed/raw/compressed phases, then injects
800-ms sender stalls and old capture timestamps. It records the coalescing count,
receiver rejection/expiry, RTT and capture-to-ACK age separately. Never treat
the latest-only policy as removal of radio latency or an event-delivery protocol.

`fault_check.py`, copied next to `client.py`, sends exactly one intentionally
corrupt benchmark frame and requires the server to close that session. It then
tries reconnection at delays 0, 0.5, 1 and 2 seconds, retaining every failure in
the JSON result. All four attempts failing is a failure, not an empty success.
The native trial reproduced an immediate reset and successful 0.5-second retry.

## Interpret And Restore

Evaluate verified count, skipped schedule slots, RTT percentiles/max, errors
and reconnect time separately. A count near 5 Hz is not a strict 5-Hz timing
guarantee. CRC success means verified application data, not zero RF retries.
Compare existing HUD PIDs, USB feedback freshness and GATT connections before,
during and after. A connected BLE link alone does not prove ambient commands
were delivered; a bonded phone alone does not prove active navigation traffic.

At completion, stop only the verified benchmark PID/session, stop discovery,
turn pairability off, and restore comma's original radio enable policy. Preserve
existing phone/ambient bonds. Remove only this trial's temporary device files.
Do not disconnect or restart the whole Navdy Bluetooth adapter for cleanup.

Only if the recorded initial scan mode was NONE (20), the native helper's
`restore-original-scan-none` command restores CONNECTABLE (21) to NONE without
resetting existing links. It refuses to overwrite a new discoverability request.
Do not use it when the initial state was different. The authorized new bond is
retained; the benchmark never deletes other bonds.

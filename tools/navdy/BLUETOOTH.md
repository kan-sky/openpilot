# Navdy Bluetooth bring-up

For USB-attached Bluetooth diagnostics, the existing private
`/data/navdy/bluetooth.json` accepts optional boolean `bluetooth_only`. When
true, only the configured paired Bluetooth peer carries display/navigation
traffic: no direct USB writer/probe and no Jetson fallback. USB ADB on port5038
remains available for logs and screenshots. A missing/disabled peer reports a
disconnected state instead of silently returning to USB. The choice is checked
every five seconds and persists across bridge/device restarts. False or omission
restores the normal USB-first policy. Preserve the existing peer/bond and write
the configuration atomically; no new pairing or radio reset is needed. Runtime
status includes `transportPolicy` separately from the actual `transport`.

Production follow-up, 2026-10-19: the paired Navdy v158 APK and Comma bridge now
support USB-first, automatic Bluetooth fallback and automatic USB return. See
[implementation and actual-device verification](../../docs/navdy_wireless_fallback_20261019.md)
and the [bounded wire protocol](WIRELESS_PROTOCOL.md). This requires the matching
Navdy APK, explicit peer configuration/bond and a Bluetooth-capable OS; a source
pull alone does not provision the other device. The earlier trials below are
historical, not the current installed state.

Earlier on 2026-10-19: native encrypted RFCOMM was measured on the actual
comma/Navdy pair, with the existing USB bridge left active. This is an isolated
synthetic benchmark, NOT a production Bluetooth HUD/navigation transport.
See [measured results and limits](../../docs/navdy_bluetooth_benchmark_20261019.md)
and [benchmark tools](bluetooth_bench/README.md). The radio was temporarily enabled
for the explicitly authorized pairing/test and restored afterward. No OS flash,
production Navdy APK replacement or vehicle-control change was made by this trial.
The 2026-10-16 inspection below is historical, not the current installed state.

The [latest-state/compression follow-up](../../docs/navdy_bluetooth_snapshot_20261019.md)
measures the same full JSON raw/compressed, bounded coalescing under sender stalls,
and receiver rejection/expiry of stale state. It remains an isolated test sink,
not a production navigation/HUD transport.

## Pinned upstream implementation

- ajouatom/openpilot `98ded8c29c140ec732d6b49081a19b0c58c99cc6`, carrot-wip:
  `launch_env.sh` requires `19.8-carrot-bt1`.
- ajouatom/agnos-builder `f5b3f77e59f1917f95e0752630ca08050463eede`:
  `patches/bluetooth-wcn3990.patch`, `userspace/usr/comma/bluetooth-radio`,
  `userspace/files/carrot-bluetooth-radio.service` and `patches/BLUETOOTH.md`.
- Source: <https://github.com/ajouatom/openpilot/blob/98ded8c29c140ec732d6b49081a19b0c58c99cc6/docs/agnos_19_8_bluetooth_trial.md>.
- Release: <https://github.com/ajouatom/agnos-builder/releases/tag/agnos-19.8-carrot-bt1>.

The upstream implementation enables native WCN3990, BR/EDR, LE and RFCOMM,
adds the Bluetooth UART and firmware initialization, and supplies BlueZ.
GPS remains on ttyHS0; Bluetooth uses ttyHS1. Never probe Bluetooth by writing
to ttyHS0. Firmware is mounted read-only from the current Bluetooth partition.
Pairing keys live in /data/bluetooth and must not enter reports or Git.

Upstream records C4 radio/pairing/restart and stationary inference tests.
Those results do not validate this WIP-FR runtime, its Jetson transport, Navdy,
or concurrent phone/ambient connections. Remote-control button mapping and
vehicle-command daemons are unrelated and must not be ported for this link.

## Observed target, 2026-10-16

Read-only SSH inspection found comma mici, AGNOS 18.4, kernel 4.9.103,
WIP-FR at cfd330a6ce944ed1e25c97b8271c33042c7810d5, boot slot A.
IsOnroad and IsEngaged were both 0 at inspection time, not a permanent safety
authorization. Existing modified files were preserved.

- CONFIG_BT is not set; no hci controller, BlueZ tools or loaded modules.
- GPS ttyHS0 exists; ttyHS1 and /dev/btpower are absent.
- bluetooth_a and bluetooth_b partitions exist. Their contents were not mounted
  or modified; an unmounted firmware directory does not prove absent firmware.
- Compared with the current branch manifest, upstream changes abl, boot and
  system. Its statement that only boot/system change compares against 19.6.3,
  NOT against our 18.4 baseline. Do not omit this bootloader difference.
- Navdy and the target Jetson were not enumerated on comma's host USB bus.

The local preflight passed eight host tests and was also executed over SSH
without installing it on comma; it reported missing native-radio prerequisites.
The existing Navdy ambient delivery and power test suites passed 3 and 5 tests
respectively. These are isolated software tests, not simultaneous radio tests.

The USB recovery check temporarily enabled the existing AdbEnabled setting.
adbd started and the gadget bound successfully, but the UDC still reported
`not attached` and the Mac saw no ADB device. The user then confirmed the cable
was in a different port, not the Jetson/data port. Physical recovery access is
therefore still unverified. The trial does not authorize flashing or rebooting
before the correct data-port connection is verified.

Run without installing or enabling anything on the target:

```sh
python3 tools/navdy/bluetooth_probe.py
python3 -m unittest discover -s tools/navdy -p 'test_*.py'
```

The probe does not scan, pair, mount firmware, open a UART, load modules,
restart a service or change files. Even controller presence is reported as
unverified, never as a working Navdy link. Unknown fields remain unknown.

## Phone and ambient coexistence

Proposed topology: phone -> existing Navdy RFCOMM service; comma <-> separate
Navdy RFCOMM service; Navdy -> existing ambient BLE/GATT controller.
No phone relay or phone hotspot is required by this topology.

Keep the phone's existing UUID/session and ambient GATT session untouched.
Use a separate authenticated, explicitly paired comma service. Pairing alone
does not establish the application protocol. Do not add generic HID controls.

The owned Navdy patches v149/v153/v154/v155/v156 contain bounded reconnect
backoff, stale-session rejection, serialized ATT writes and OFF priority.
Preserve these. A peripheral power-cycle recovery recorded in v156 is not
proof that three simultaneous links work, nor proof of bandwidth exhaustion.

For an initial data-only experiment, limit/coalesce state updates, bound queues
and apply monotonic freshness expiry. Do not send imagery over this new link.
Stop discovery after setup and use backoff for the selected peer. A failed
comma connection must never reset Navdy's whole Bluetooth adapter: that would
also disconnect the phone and ambient controller. Inspect existing bridge
recovery logic on the actual deployed version before enabling the new link.

## Deployment gates

1. Establish physical recovery access, save the installed code/dirty changes,
   current OS metadata and rollback path. USB cable presence alone is not
   successful ADB/fastboot access. Do not force onroad to test this transport.
2. Review the complete device-specific manifest, updater, bootloader difference
   and runtime dependencies. Verify downloaded image hashes. Do not change
   launch_env.sh alone: that can trigger a fleet-wide OS update on Git pull.
3. Preserve the current known-good slot. Only a deliberately approved trial
   may flash/activate the inactive slot; never overwrite the rollback slot.
   Account for the old code's OS-version gate before rebooting or rolling back.
4. Verify radio readiness on the new OS while offroad, then GPS, cameras,
   model runtimes, USB/Jetson and normal reboot/reconnect behavior separately.
5. Pair only the selected Navdy. Validate the dedicated protocol before enabling
   navigation reception. Test the real phone + Navdy + ambient + comma together,
   including each peer disconnect/reconnect, sustained state changes, delayed
   packets, stale data and ambient OFF acknowledgements.

Hardware availability, installed OS, controller power, pairing, application
data and safe navigation integration are separate milestones. None is implied
by merely adding a settings item or passing host-side tests.

# WIP-FR Accumulated Changes

Publish the completed changes on top of `5378d124` to
`leehyuk1108/carrotpilot:WIP-FR`. This is a source publication, not a new
device installation, reboot, APK distribution or driving-validation run.

## Included

- Traverse display-speed conversion without rescaling chassis motion data.
- Scoped, driver-armed manual stop hold and its matching Panda safety source.
- Explicit Trailblazer NNFF substitution for Traverse when NNFF is enabled.
- Navdy Bluetooth transport, USB fallback, optional Bluetooth-only operation,
  bounded display expiry, ambient door lifecycle and display power handling.
- Paired Jetson USB Internet and Navdy management helpers.
- Carrot Vision host-only ICE for LAN/Tailscale.
- USB bench power exclusion from vehicle low-voltage shutdown.
- Focused tests, protocol tools and reports for these changes.

## Verification Before Publication

- Local Navdy/Jetson/Bluetooth-tool run: 250 passes; five DBC tests initially
  blocked by the missing desktop `params_pyx` extension. All 16 ambient tests,
  including those five, subsequently pass against the installed native Comma
  modules in an isolated process with no CAN publication.
- Local Traverse display/manual-hold run: 339 passes, including native C safety
  hooks compiled for the desktop. This is not MCU or physical brake validation.
- Local power-policy run: 50 passes with platform dependencies stubbed.
- Installed native NNFF run: 18 passes using in-memory Params, no CAN commands
  or IPC publication. This does not validate donor-model steering on the road.
- Installed WebRTC run: five passes, including a local peer-to-peer data-channel
  exchange without external STUN. No camera or live vehicle-message stream used.
- Syntax/JSON checks and staged whitespace checks pass. Private-key/token
  pattern scanning of the selected files reports no findings.

The 23 installed changed runtime files checked by SHA256 match the publication
sources. `gm/values.py` additionally contains pre-existing device-only mass and
center-to-front calibration; those values remain on the device and are not
silently imported into Git. The new Jetson-side management helper is published
as source, not claimed to be installed under the Comma checkout.

Private credentials, pairing configuration, raw logs, deployment backups,
generated binaries, browser captures and the old standalone layout preview
remain excluded. Navdy APK patches are maintained in a separate repository;
this publication includes the Comma-side protocol and compatibility notes, not
an APK installer. See the individual reports for prior device measurements and
remaining physical/long-duration limitations.

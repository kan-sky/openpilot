# Wayon/sunnypilot BSM and DM Attribution

The C4 `new ui` blind-spot presentation in
`openpilot/selfdrive/ui/mici/onroad/blind_spot_indicators.py` is adapted from
`selfdrive/ui/sunnypilot/onroad/blind_spot_indicators.py` in Wayon/sunnypilot.
The following upstream repository license is retained verbatim.

The NewUi DM presentation also reuses the already ported
`openpilot/selfdrive/ui/mici/onroad/driver_state.py` renderer and the four
`icons_mici/onroad/driver_monitoring/` person, cone, background and phone assets.
Their drawing routines and asset bytes match the Wayon source at
`da8cda353ec2a16bcc6d3d870f54137b809af3ff`; the NewUi adapter adds display-only
freshness and overlap guards without changing monitoring decisions.

This project uses software from Haibin Wen and SUNNYPILOT LLC and is licensed under a custom license requiring permission for use.

This software is licensed under a custom license requiring permission for use.

## Custom MIT License

Copyright (c) 2024, Haibin Wen, SUNNYPILOT LLC

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to view and modify the Software, subject to the following conditions:

1. **Permission Required**: Permission Required for Commercial, For-Profit, or Closed Source Use: Use of the Software, in whole or in part, for any commercial purposes, for-profit projects, or in closed source projects requires explicit written permission from the original author(s).

2. **Redistribution**: Any redistribution of the Software, modified or unmodified, must retain this license notice and the following acknowledgment:
   "This software is licensed under a custom license requiring permission for use."

3. **Visibility**: Any project that uses the Software must visibly mention the following acknowledgment:
   "This project uses software from Haibin Wen and SUNNYPILOT LLC and is licensed under a custom license requiring permission for use."

4. **No Warranty**: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Contact sunnypilot Support <support@sunnypilot.ai> for permission requests.

---

Haibin Wen, SUNNYPILOT LLC

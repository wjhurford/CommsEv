# CommsEv Console

`app.py`, PySide6. Run it from the repo root: `python3 console/app.py`,
`console\CommsEv Console.bat` on Windows, or `docker compose up` for a browser.
A crash is written to `console/last_error.log`.

The Console is a client. The model lives in `tools/stub_telemetry.py`, which
the Console runs as a background process and draws; if a number on screen
looks wrong, look there first. The optional ROS 2 bridge (`ros2/`) is started
in WSL on Windows and in plain `bash` elsewhere.

Sidebar, in the order a run happens: **Setup** (scene, blue fleet, red fleet,
points — everything else stays greyed out until the run exists) · **System** ·
**Mission** · **Network** (authority and measured topology, live) ·
**Comms / Contested** (emitters, links, jammers, noise floor) · **Results**
(sweeps, plots, click a row to replay). Map in the centre; three scoped
terminals below it — white / blue / red — plus the log. `../COMMANDS.md` is
the grammar. Console tests run headless with the rest of `tests/test_all.py`.

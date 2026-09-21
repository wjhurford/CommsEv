# CommsEv Console

`app.py`, PySide6. Run it from the repo root: `python3 console/app.py`,
`windows\CommsEv Console.bat` on Windows, or `docker compose up` for a browser.
A crash is written to `console/last_error.log`.

The Console is a client. The model resides in `tools/stub_telemetry.py`,
which the Console runs as a background process and renders; a value that
appears incorrect on screen should be traced there first. The optional ROS 2
bridge (`ros2/`) is started in WSL on Windows and in plain `bash` elsewhere.

Sidebar, in the order a run happens: **Setup** (scene, blue fleet, red fleet,
points — no other tab is enabled until the run exists) · **System** ·
**Mission** · **Network** (authority and measured topology, live) ·
**Comms / Contested** (emitters, links, jammers, noise floor) · **Results**
(sweeps, plots; a row replays its cell). Map in the centre; three scoped
terminals below it — white / blue / red — plus the log. `../docs/COMMANDS.md` is
the grammar. Console tests run headless with the rest of `tests/test_all.py`.

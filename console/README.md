# Deadband Console

The application. Double-click **Deadband Console.bat**.

## First run only

Install the one dependency, using your **Windows** Python (not WSL):

```
pip install PySide6 pyyaml
```

## What it does now

- Loads `scenarios/lab_box_two_agents.yaml` on startup
- Scenario tree on the left, properties on the right, log below
- Unsourced parameters shown in amber, with a running count in the status bar
- Three fixed views: TOP, FRONT, ISO. Scroll to zoom, drag to pan
- **Run** launches the telemetry source as a background process and animates
  the agents from it

## What it does not do

No physics, no radio modelling, no 3D engine. The Console is a client over the
scenario file and the telemetry stream. When the real simulator replaces the
stub, only one line in `app.py` changes — `start_run()`.

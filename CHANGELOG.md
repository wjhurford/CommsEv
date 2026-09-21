# Changelog

All notable changes. Dates are release dates; the format follows
[Keep a Changelog](https://keepachangelog.com/).

## [Unreleased]

## [0.2.0] — 2026-09-18

First public release.

### Added
- Console in a browser: `Dockerfile`, `docker-compose.yml`, published image
  at `ghcr.io/wjhurford/commsev`. The build runs the full test suite.
- `docs/TUTORIAL.md`: clone to your own experiment in thirty minutes.
- `CITATION.cff`, `CONTRIBUTING.md`, `NOTICE`, CI on every push.
- Validator accepts a fleet that declares no `authority` (fleets defer that
  decision to Setup by design); pinned by a test.

### Changed
- Renamed from **Deadband** to **CommsEv** (Communications Evaluator):
  package `commsev`, ROS package `commsev_ros`, CLI `python3 -m commsev`,
  env var `COMMSEV_ROOT`.
- READMEs cut to essentials; per-patch notes and superseded docs moved to
  `docs/history/`.
- The Console's shell launcher is platform-aware: WSL on Windows, `bash`
  elsewhere (including the Docker image).

### Fixed
- ROS repo-root discovery looked for a `scenarios/` folder that no longer
  exists.
- The ROS cleanup `pkill` pattern did not match after the rename.

## [0.1.0] — 2026-09-14

Internal. The Deadband testbed as handed over at the end of the Loughborough
LUCAS Lab internship: RF/jamming/authority/drift model, PySide6 Console,
headless sweep harness, ROS 2 bridge, 554 tests.

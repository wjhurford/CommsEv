# Contributing to CommsEv

CommsEv is a research testbed. The bar for a change is not "does it run" but
"can a result produced with it still be defended". Everything below follows
from that.

## Before you start

```bash
python3 tests/test_all.py        # must pass before you touch anything
```

Read `CLAUDE.md` (the rules, in the shortest form), `docs/vocabulary.md` (the
three layers), and the top three items of `docs/REVIEW-2026-09-01.md` (the
known open issues) before changing the model.

## The rules that are not up for negotiation

1. **Provenance.** Every physical quantity in a layer file or in
   `tools/stub_telemetry.py` is `{value, unit, source}`. If you add a constant,
   add a row to `SOURCES.md` saying where it came from — or that it is a free
   parameter. An empty source is allowed and is counted; a plausible-looking
   invented one is not.
2. **Missions never import `rclpy`.** Mission behaviour scripts take plain
   numbers and return plain numbers. The ROS node wraps them.
3. **`target(agent, world)` is frozen.** Add new senses to `world`.
4. **Authority ≠ routing.** Keep them as independent keys everywhere.
5. **Missions are blue-only.** Red is jamming and interception.
6. **Comparative claims only.** A doc, a commit message or a plot title that
   quotes a CommsEv dB figure or PDR as a measured truth will be asked to
   reword.

## Making a change

- One concern per commit. The commit message says what changed and why; the
  test that pins it says how you know.
- **If you change behaviour, add a test that would have caught the old bug.**
  The suite is one file, `tests/test_all.py`, no pytest. Each test is a
  function that calls `check(name, condition, note)`; register it in the list
  at the bottom.
- The model lives in `tools/stub_telemetry.py`. The Console
  (`console/app.py`) only draws what the model streams; do not put physics in
  the GUI.
- Layer files (`scenes/`, `fleets/`, `missions/`, `agents/`) are data. A new
  scene or agent is welcome; a fleet is a *decision* and is normally built in
  the Console rather than hand-written (see `fleets/README.md`).
- Docs that describe superseded behaviour are banner-marked **SUPERSEDED**
  rather than deleted, so the history of a decision stays readable. Do the
  same.

## Finishing

```bash
python3 tests/test_all.py        # same count or higher, zero failures
python3 -m commsev validate default_run.yaml
```

Open a pull request against `main`. Say what question the change lets the
testbed answer that it could not before.

## Licence

By contributing you agree that your contribution is licensed under the
Apache License 2.0, the same as the rest of the project.

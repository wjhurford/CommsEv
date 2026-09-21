# Contributing

The acceptance criterion for a change is not that it runs but that a result
produced with it can still be defended. Accordingly:

- `python3 tests/test_all.py` before and after. Same count or higher, zero
  failures. The suite is one file, no pytest: each test calls
  `check(name, condition, note)` and is registered in the list at the bottom.
- A change in behaviour is accompanied by a test that would have detected the
  previous behaviour.
- A new constant is accompanied by a row in `SOURCES.md` giving its source,
  or declaring it a free parameter. An empty source is permitted; an invented
  one is not.
- Physics belongs in `tools/stub_telemetry.py`, never in the Console.
- Missions never import `rclpy`. `target(agent, world)` is frozen. Authority
  and routing stay separate keys. Missions are blue-only.
- No dB or PDR value produced by CommsEv is quoted as a measurement, in code,
  documentation or plots.
- A superseded document is banner-marked and moved to `docs/history/`, not
  deleted.

Before starting, read `CLAUDE.md`, `docs/vocabulary.md` and the opening items
of `docs/REVIEW-2026-09-01.md`. One concern per commit. Pull requests are made
against `main` and state which question the change allows the testbed to
answer. Contributions are licensed under Apache-2.0, as is the rest of the
project.

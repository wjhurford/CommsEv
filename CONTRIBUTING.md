# Contributing

The bar is not "does it run" but "can a result produced with it still be
defended". So:

- `python3 tests/test_all.py` before and after. Same count or higher, zero
  failures. The suite is one file, no pytest: each test calls
  `check(name, condition, note)` and is registered in the list at the bottom.
- If you change behaviour, add a test that would have caught the old bug.
- If you add a constant, add a row to `SOURCES.md` — its source, or that it
  is a free parameter. Empty is allowed; invented is not.
- Physics goes in `tools/stub_telemetry.py`, never in the Console.
- Missions never import `rclpy`. `target(agent, world)` is frozen. Authority
  and routing stay separate keys. Missions are blue-only.
- Never quote a CommsEv dB or PDR as a measurement, in code, docs or plots.
- A superseded doc is banner-marked and moved to `docs/history/`, not deleted.

Read `CLAUDE.md`, `docs/vocabulary.md` and the top of
`docs/REVIEW-2026-09-01.md` first. One concern per commit. Pull requests
against `main`; say what question the change lets the testbed answer.
Contributions are licensed under Apache-2.0 like the rest of the project.

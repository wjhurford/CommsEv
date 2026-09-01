# Vocabulary — the three layers

This supersedes the earlier "a run is a MAP plus a MISSION" decision in
`docs/BACKLOG.md`. Agreed with Will, 1 Sep 2026 (revised same day: the middle
layer is called a **fleet**, not a "scenario" — "scenario" was the word causing
the confusion, so it is retired from the framework entirely).

```
  scene  +  fleet   ──composed by──►   mission
  world     agents                     the command
```

## scene — the world only

The arena, the radio medium, the contested-environment background (spectrum,
GNSS, wind, propagation), and named points of interest. **No agents, no
networks, no tasking.** A scene answers *where*, never *who* or *what*.

Lives in `scenes/`. `kind: scene`. Terminal — a scene references nothing.
This is what you pick from the **File** dropdown.

## fleet — the agents and their wiring

Which agents exist, their bodies, sensors and radios, which networks they
belong to, and each network's authority, routing and squads. **No world, no
tasking.** A fleet answers *who*, never *where* or *what*.

Lives in `fleets/`. `kind: fleet`.

Known coupling, declared honestly: spawn poses are coordinates, which ties a
fleet to scenes shaped like the ones it was written for. Spawning at a scene's
named points is the portable form, queued.

## mission — the command, and nothing else

Objectives for agents. A mission names **no scene and no fleet** — it is cut
off from both, deliberately: the same command can be issued to any fleet on
any scene that resolves its points. A mission answers *what the fleet is
doing*, and it is **issued, not opened**: with the run up, in the terminal —

```
SETMISSION test
blue launch
```

`SETMISSION <name>` applies `missions/<name>.yaml`'s objectives to the running
fleet, **gated by command authority** (an agent whose decider cannot reach it
is not retasked — contested comms gate what you can command), and titles the
run: kept bags become `runs/<mission>_<timestamp>`, CSV exports default to
`<mission>.csv`. `REOBJECTIVE <agent> ...` still adjusts one agent live.
REMISSION and LOADMISSION are gone; SETMISSION is the one order verb.

Lives in `missions/`. `kind: mission`, `objectives:` only. The
`missions/*.py` files are **behaviour scripts** an objective can invoke
(`wall_follow`, `pursue`) — verbs, not run-files.

```yaml
kind: mission
objectives:
  car1: {do: shuttle, between: [A, B]}
```

## Composing a run

A run is **scene + fleet**, composed in the Console's **Setup tab**: choose a
scene (the world appears), choose a fleet (a spawn dialog asks where the
agents start, with the fleet's own poses as defaults). Setup writes the
composition to `runs/current_setup.yaml` — a real file, so the run stays
diffable and re-runnable — and headless tools use `default_run.yaml` (the lab
scene + lab fleet) when nothing else is named.

## The merge rule (one rule, every layer)

`resolve_mission()`/`resolve_doc()` resolve `scene:`, overlay `fleet:`,
overlay the composing file itself (its spawn overrides, etc.). Upper layers
win per top-level key; `agents` merge per-id. Everything downstream receives
one flat dict shaped exactly like the old self-contained file, so the sim,
the Console and PlotJuggler are unchanged.

## Backward compatibility, and the attic

A file that names no base is **self-contained** — the old shape — and still
loads. `map:` remains a silent alias for `scene:`. The old `scenarios/` files,
the old fleet-embedding scenes and the old yaml missions are preserved in
`attic/` (moved, not deleted — several carried uncommitted work). Nothing in
the framework reads `attic/`; delete it whenever you are sure.

The canonical working set is exactly one of each:
`scenes/lab_box.yaml` + `fleets/3_roboracer.yaml` + `missions/test.yaml`,
plus `default_run.yaml` (the headless scene+fleet composition).

# Cells and the Network tab

Built together (1 Sep) because they are two halves of one idea: the **Network
tab** shows the command structure, and the **cells** are how you act within
it. One without the other is half a picture.

## The three cells (embedded terminals)

Modelled on a real exercise's control cells:

- **Blue cell** — the friendly commander. May issue mission and manoeuvre
  orders to the BLUE network only: `SETMISSION <name>`, `blue launch` /
  `blue halt`, `REOBJECTIVE <blue-agent> …`. A red-side command is refused
  with a printed reason.
- **Red cell** — the adversary. May command the RED network only:
  `red launch` / `red halt`, `JAM <id> power/band …`. It has **no** SETMISSION
  (missions are a friendly manoeuvre concept) and cannot task blue.
- **White cell** — the umpire. Anything, any side, plus the plain shell. Extra
  "New terminal" tabs are white cells.

Bash works in every cell — they are still terminals; only the tactical verbs
are scoped. Scope is resolved by `Console.side_of()`: a network's `system`
field (`friendly`→blue, `adversary`→red) decides the side of that network and
of every agent on it. A cell that cannot place a scope on its own side refuses
rather than sending, so you cannot fat-finger a blue order from the red cell.

Why scope at all: it makes the exercise honest. A blue operator in the field
does not get to switch the jammer off; a red operator does not get to retask
blue's cars. Enforcing it in the UI means a two-person (or two-window) exercise
behaves like the real thing, and it is the frame that command interception
(next) plugs into — the red cell hears, the red cell acts, all within remit.

## Missions are blue-only

A mission is a friendly manoeuvre plan; the red side is jamming, driven by
launch/halt/JAM, never by objectives. Enforced in the sim too (not just the
UI): `_is_taskable()` refuses an objective for any adversary-network agent or
any jammer platform, so SETMISSION skips it and a live REOBJECTIVE on it is
ignored — belt and braces with the cell scoping.

## The Network tab

`netcheck.py`'s tables, live in the GUI — the command structure that has been
computed every frame (`command_authority`, `observed_topology`) but was
invisible.

- **Declared** — per network: authority, routing, coordinator, squads, and
  the leader-loss doctrine (below). What the file says.
- **Live — who decides for whom** — each agent's decider, tier and whether
  that decider is reachable *right now*. Under jamming, agents whose decider
  is cut off turn red here — the command cost of the spectrum, made explicit.
- **Measured topology** — the shape the graph ACTUALLY forms (star / mesh /
  tiered), the hub, betweenness, and active/spare/down link counts. If the
  measured shape disagrees with the declared routing, that gap is flagged —
  it is a finding, not a bug (you can declare mesh over a physical star).

## Editable doctrine — leader-loss (the discussion Will flagged)

When a squad's leader becomes unreachable, what happens is now a **declarable
per-network choice**, `leader_loss`, not a hard-coded rule:

- `fallback` (default) — the squad reports up to the coordinator: degraded,
  not decapitated. The robust middle ground.
- `strand` — the squad is on its own the instant its leader drops; no
  automatic reach-up. Models a strict chain of command (or a doctrine where
  silence means hold), and is the more brittle, more realistic-for-some-forces
  choice.

Shown in the Network tab's Declared section. This is exactly the kind of
command-resilience choice the framework exists to let you *compare* — run the
same jamming against `fallback` and `strand` and measure which keeps more of
the fleet under command. Making it a field, not code, is the point.

**Open design question for later:** doctrine is currently edit-in-file
(change `leader_loss:` in the fleet and reload). A live editor (like the
jammer power control) is a natural follow-up, but doctrine changing mid-run
is a stronger claim than jammer power changing mid-run — a force does not
usually rewrite its command doctrine in contact — so it is deliberately
file-level for now. Worth deciding together whether live doctrine switching
is a capability we want or a realism we should withhold.

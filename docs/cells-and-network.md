# Cells and the Network tab

Built together (1 September 2026) because they are two halves of one idea: the
**Network tab** shows the command structure, and the **cells** are the means of
acting within it. Either alone is half a picture.

## The three cells (embedded terminals)

Modelled on a real exercise's control cells:

- **Blue cell** — the friendly commander. May issue mission and manoeuvre
  orders to the blue network only: `SETMISSION <name>`, `blue launch` /
  `blue halt`, `REOBJECTIVE <blue-agent> …`. A red-side command is refused
  with a printed reason.
- **Red cell** — the adversary. May command the red network only:
  `red launch` / `red halt`, `JAM <id> power/band …`. It has **no** SETMISSION
  (missions are a friendly manoeuvre concept) and cannot task blue.
- **White cell** — the umpire. Anything, any side, plus the plain shell. Extra
  "New terminal" tabs are white cells.

Bash works in every cell — they remain terminals; only the tactical verbs are
scoped. Scope is resolved by `Console.side_of()`: a network's `system` field
(`friendly`→blue, `adversary`→red) decides the side of that network and of
every agent on it. A cell that cannot place a scope on its own side refuses
rather than sending, so a blue order cannot be issued by mistake from the red
cell.

The purpose of scoping is to keep the exercise faithful. A blue operator in the
field cannot switch the jammer off; a red operator cannot retask blue's cars.
Enforcing it in the UI means a two-person (or two-window) exercise behaves like
the real thing, and it is the frame that command interception (the next step)
plugs into — the red cell hears, the red cell acts, all within its remit.

## Missions are blue-only

A mission is a friendly manoeuvre plan; the red side is jamming, driven by
launch/halt/JAM, never by objectives. This is enforced in the simulation as
well as in the UI: `_is_taskable()` refuses an objective for any
adversary-network agent or any jammer platform, so SETMISSION skips it and a
live REOBJECTIVE on it is ignored — a second guard alongside the cell scoping.

## The Network tab

`netcheck.py`'s tables, live in the GUI — the command structure that has been
computed every frame (`command_authority`, `observed_topology`) but was
previously invisible.

- **Declared** — per network: authority, routing, coordinator, squads, and
  the leader-loss doctrine (below). What the file says.
- **Live — who decides for whom** — each agent's decider, tier and whether
  that decider is reachable *at this moment*. Under jamming, agents whose
  decider is cut off turn red here — the command cost of the spectrum, made
  explicit.
- **Measured topology** — the shape the graph actually forms (star / mesh /
  tiered), the hub, betweenness, and active/spare/down link counts. If the
  measured shape disagrees with the declared routing, that gap is flagged —
  it is a finding, not a bug (mesh may legitimately be declared over a
  physical star).

## Editable doctrine — leader-loss

When a squad's leader becomes unreachable, what happens is now a **declarable
per-network choice**, `leader_loss`, not a hard-coded rule:

- `fallback` (default) — the squad reports up to the coordinator: degraded,
  not decapitated. The robust middle ground.
- `strand` — the squad is on its own the instant its leader drops; no
  automatic reach-up. Models a strict chain of command (or a doctrine where
  silence means hold), and is the more brittle choice, though more realistic
  for some forces.

Shown in the Network tab's Declared section. This is exactly the kind of
command-resilience choice the framework exists to *compare* — run the same
jamming against `fallback` and `strand` and measure which keeps more of the
fleet under command. Making it a field rather than code is the design intent.

**Open design question for later:** doctrine is currently edit-in-file
(change `leader_loss:` in the fleet and reload). A live editor (like the
jammer power control) is a natural follow-up, but doctrine changing mid-run
is a stronger claim than jammer power changing mid-run — a force does not
usually rewrite its command doctrine in contact — so it is deliberately
file-level for now. Whether live doctrine switching is a capability worth
having or a realism worth withholding remains to be decided.

# Patch 6 — what changed, and how to check it

Three commits, applying on top of `fc9f219`.

---

## 1. Two-squad hierarchy (`808e303`)

### Changes
- **`scenes/squad_hall.yaml`** — new. 30x20 hall, gcs + six cars in two squads
  (alpha: car1 leads car2/car3; bravo: car4 leads car5/car6). Exists because
  hierarchy is only structurally distinct from a star at **two or more** squads.
- **`missions/squad_patrol.yaml`** — new. Tasks both squads so the hierarchy has
  something to command.
- **`scenes/lab_box.yaml`** — on the new schema (`authority` / `routing` /
  `squads`) with **one** squad, which is what three real cars honestly support.
- **Bug fixed:** `command_authority()` never read the `squads` block, so every
  agent reported straight to the coordinator — a hierarchy in the comms routing
  and none in the command structure.
- `command_authority()` now reads `authority:`; `architecture:`/`topology:` kept
  as legacy aliases.

### Check
1. `python3 tests/test_all.py` — expect **50 passed, 0 failed**.
2. `python3 tools/netcheck.py missions/squad_patrol.yaml`
   Expect: seven agents; car2/car3 report to **car1**, car5/car6 to **car4**,
   car1/car4 to **gcs**; shape `mixed`, betweenness ~0.69, hub `gcs`; active
   edges are gcs-car1, gcs-car4 and each leader to its members, with **no**
   cross-squad edge.
3. Open `missions/squad_patrol.yaml` in the Console — six cars in two groups,
   patrolling opposite halves.
4. `python3 tools/netcheck.py scenes/lab_box.yaml` — one squad, so `tiered`
   correctly measures as a star on the squad leader. **That is not a bug: one
   squad IS a star.**

---

## 2. Patch 5 checklist (`a2c1a3a`)

`docs/history/PATCH-05-CHECKS.md`, plus the standing policy: every patch ships a
checklist stating what changed, what to check in the Console, what to check
outside it, and **explicitly what is not visible**.

---

## 3. Tests, inspector, history fix (`051c45c`)

### Changes
- **`tests/test_all.py`** — 50 tests, no pytest needed. Each maps to a bug that
  actually shipped.
- **`tools/netcheck.py`** — one command for the whole network picture.
- **Console:** objective history clears when a run stops.

### Check
5. `python3 tests/test_all.py` → 50 passed.
6. `python3 tools/netcheck.py` → the default fleet's picture.
7. `python3 tools/netcheck.py --budget` → link budget vs distance. Expect `up`
   to ~100 m, `degraded` ~200 m, `down` by 500 m.
8. **History clears:** start a run, `REOBJECTIVE car1 pursue car3`, stop the
   run. The `history` folder should be back to one row (`t=0.0 ... (initial)`),
   not carrying the retask from the previous run.

### Corrections to the patch 5 checklist
- Checks 9 and 11 **did not work as written** — the snippets were indented
  inside the markdown, and bash preserves that indentation, so Python threw
  `IndentationError`. Check 11 also had a stray leading `|`. Use
  `tools/netcheck.py` instead; it replaces both.
- **"links 6 (0 degraded)" is in the status bar** at the very bottom of the
  window, below the Output/Terminals dock. It is easy to miss.

---

## Still not visible anywhere in the Console

Unchanged from patch 5, and now the top of the queue:

- Command authority (who decides, reachable, tier)
- Measured topology (shape, betweenness, hub)
- SINR / rx power / PDR per link
- `active` vs `spare` — the viewport draws both identically

`netcheck.py` makes these *reachable* but not *visible*. Surfacing them belongs
before the jamming work: jamming you cannot see is a demo you cannot debug.

---

## New ideas — from Will's README (brother in industry)

Sorted by what I would actually do first.

### Act on now
- [ ] **Tests** — DONE this patch. He was right; three regressions had already
      shipped that a test would have caught instantly.
- [ ] **Repo is now private** — I clone over HTTPS with no credentials. Read
      access may disappear. If a future session says "repository not found",
      that is why, and the fix is handing files over directly.
- [ ] **Docker + browser-accessible** — "clone, one command, it runs" is the
      difference between a tool others can use and one only Will can run. Kills
      the WSL/Windows path pain. High value for the internship hand-over.
- [ ] **CLAUDE.md and skills in the repo** — so a new session starts with the
      conventions instead of rediscovering them.

### The strongest research idea in the document
- [ ] **Gamified jamming ladder.** Levels of progressively harder jamming,
      graded on time-to-solve and network quality, run in BOTH directions
      (defend a network; attack a hardened one). Level 1 one band jammed;
      level 2 adds an adversarial agent targeting specific frequencies; and on
      until the network resists continuous, reactive, random and deceptive
      jamming. This is a **benchmark**, and benchmarks get adopted. The four
      jammer classes come straight from the papers, so the levels are
      literature-grounded rather than arbitrary.

### Showcase scenarios — build toward, not next
- [ ] **Deep strike recce** — drones find a target under jamming, relay the
      coordinates they *believe* are right, a missile is sent: were they
      correct? A perfect demonstration of the per-agent-belief work.
- [ ] **Missile intercept** — track and neutralise an inbound under jamming.
- These need targets, weapons, tracking and engagement — a lot of new
      machinery. Held as the showcase, after jamming is validated. Building
      them first would rest a demo on an unvalidated comms model.

### Capability modelling
- [ ] **Per-equipment performance** — bespoke radios: directional antennas,
      frequency agility, jam resistance, fibre-optic tether. Makes "which radio
      should we buy" answerable, which is what makes it useful to a military.
- [ ] **Self-jamming at scale** — 8+ drones in a coordinated strike interfering
      with each other. Falls out of the SINR model almost free once every agent
      is an emitter, and it is a real operational problem.
- [ ] **Single-message latency** as a first-class metric.
- [ ] **Encryption** — cost in overhead and latency; key distribution as a
      failure mode.
- [ ] **ML / sensor fusion solutions** to the jamming levels, so unique
      approaches can be graded against each other.

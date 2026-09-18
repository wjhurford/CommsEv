# Patch 5 — what changed, and how to check it

Applied at `fc9f219`. Two commits. This is the walk-through: every change, what
you should see, and — just as important — what you will NOT see, because a lot
of patch 5 is under the hood and has no Console surface yet.

**Read the "Known gaps" section before hunting for anything.** Several changes
are real, tested, and completely invisible in the GUI. That is expected, not a
fault, and the fix is on the list.

---

## Commit 1 — architecture, objective history, tab order

### Changes
- `command_authority()` added: for every agent, every frame, works out who
  decides for it and whether that decider is reachable. Three modes:
  `centralized`, `decentralized`, `hierarchical`.
- Networks carried into the world dict so architecture is readable at frame time.
- Mission tree: each agent gains a `history` folder logging every objective it
  has been given, with the sim time it took effect.
- Tab order fixed: the sidebar renders in the order tabs are added, so
  Environment is now genuinely first.
- Restored the `link_state` definition clobbered by an earlier edit.

### Check in the Console
1. **Tab order.** Left sidebar, top to bottom, should read:
   `Environment, Overview, Mission, Comms, Cyber, Results`.
   *Previously Environment was at the bottom.*
2. **Objective history.** Start a stub run. Open the **Mission** tab, expand an
   agent — there should be a `history` folder under it with one greyed row:
   `t=0.0  shuttle   (initial)`.
3. **History logs a retask.** In a terminal, `REOBJECTIVE car1 pursue car3`.
   The `objective:` row should change to `pursue car3` AND a new greyed row
   should appear under `history`: `t=<something>  pursue car3`.
4. **Retask twice.** `REOBJECTIVE car1 shuttle A B`. A third history row.
   History should never overwrite, only append.

### Check outside the Console
5. Command authority is **not displayed anywhere**. To see it at all:
   ```bash
   python3 tools/stub_telemetry.py --scenario scenarios/three_car_fleet.yaml \
     | head -1 | python3 -m json.tool | grep -A4 authority
   ```
   Expect every agent to show `decider: gcs, reachable: true,
   tier: coordinator` (three_car_fleet is centralized).

---

## Commit 2 — SINR link model, emergent reachability, routing

### Changes
- `rf_link()`: real link budget. Log-distance path loss with a Friis 1 m
  reference, SINR with an interference term, logistic packet delivery ratio.
  One currency for range, walls and (later) jamming.
- `link_state()` now delegates to it — every caller gets real physics.
- Links are now **candidates** generated from network membership alone. The old
  code built them from whether a `coordinator` existed, so the topology was
  derived from the declaration and could never contradict it.
- `apply_routing()`: `star` / `mesh` / `tiered` decide which reachable
  candidates are **active**. Every link now carries `active` (used by this
  routing) and `usable` (physically up).
- `observed_topology()`: measures what the topology actually IS — betweenness,
  shape (star/mesh/mixed), and which node is the hub — reported next to the
  declared routing.
- Betweenness normalisation fixed: counting adjacent pairs capped a 4-node star
  at 0.5 and made it look like a partial mesh.

### Check in the Console
6. **Expect MORE lines than before, and this is a regression in clarity.**
   `three_car_fleet` used to draw 3 links (each car to the gcs). It will now
   draw **6** — all pairs — because spare links are real objects now and the
   viewport does not yet know that `active` exists. Status bar should read
   `links 6 (0 degraded)` where it used to read `links 3`.
   *This is expected. The fix (draw active solid, spare faint) is on the list.*
7. **Everything should still be "up".** With the honest RF model, 2.4 GHz at
   20 dBm reaches roughly 200 m before degrading. An 8x8 room is comprehensively
   in range, so `0 degraded` is correct. The old model faded at 7 m and died at
   12 m, which was inventing failures that do not exist.
8. **Nothing else changed on screen.** Cars drive as before; missions,
   retasking, sensor view, properties all unaffected.

### Check outside the Console
9. **Link budget is physically sane:**
   ```bash
   python3 -c "
   import sys; sys.path.insert(0,'tools')
   from stub_telemetry import rf_link
   a={'x':0,'y':0,'z':0}
   for d in (1,10,100,200,500):
       b={'x':d,'y':0,'z':0}; r=rf_link(a,b)
       print(d,'m ->',r['rx_dbm'],'dBm  SINR',r['sinr_db'],' ',r['state'])"
   ```
   Expect about `-20 dBm` at 1 m, `up` out to ~100 m, `degraded` around 200 m,
   `down` by 500 m.
10. **Topology is measured, not assumed:**
    ```bash
    python3 tools/stub_telemetry.py --scenario scenarios/three_car_fleet.yaml \
      | head -1 | python3 -c "import sys,json; print(json.load(sys.stdin)['topology'])"
    ```
    Expect `shape: star`, `max_betweenness: 1.0`, `hub: gcs`, `edges: 3`.
11. **Active vs spare:**
    ```bash
    python3 tools/stub_telemetry.py --scenario scenarios/three_car_fleet.yaml \
      | head -1 | python3 -c "
    import sys,json
    for l in json.load(sys.stdin)['links']:
        print(l['a'],l['b'],l['state'],'ACTIVE' if l['active'] else 'spare')"
    ```
    Expect 3 ACTIVE (all touching gcs) and 3 spare (car-to-car).

---

## Known gaps — real, tested, and INVISIBLE in the Console

None of the following is displayed anywhere in the GUI. They exist only in the
telemetry frame. Do not go looking for them on screen.

- Command authority (who decides for each agent, whether reachable, which tier)
- Observed topology (shape, betweenness, hub)
- SINR, rx power, PDR per link
- `active` vs `spare` distinction — the viewport draws both identically
- Declared vs measured mismatch warnings

**Consequence:** the whole point of Steps 1-2 — being able to see that a
declared architecture does not match the measured one — currently requires
reading raw JSON. Surfacing this is the next Console job and it belongs before
the jamming work, because jamming without a way to see its effect is a demo you
cannot debug.

---

## Experiments worth running now

**E1 — Blast radius** (works today, no jammer, no simulated time). Remove each
node in turn, count how many others lose command authority.
Over `squad_hall` (patch 6): centralized worst 6/6 mean 0.86; hierarchical
worst 2/6 mean 0.29; decentralized 0/6. Hierarchy bounds the damage.

**E2 — Routing shapes.** Set `routing:` to star, then mesh, then tiered in a
scene and check `observed_topology` matches what you declared. A single squad
under `tiered` correctly measures as a **star centred on the squad leader** —
that is not a bug, one squad IS a star.

**E3 — Does the room break links?** Drive the fleet to opposite corners and
watch link state. Expect it never to degrade: the room is too small. This is
the finding that makes the jammer load-bearing rather than optional.

---

## Standing policy from here

Every patch ships with this document: bullet-point every change, state what to
check in the Console, state what to check outside it, and state explicitly what
is NOT visible. Anything that works for the simulator but not for the person
using it is not finished.

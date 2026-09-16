# CommsEv — handover and demo script

For the incoming PhD student. Written 11 September 2026, against the working
tree at **522 tests passing**.

Everything in this document has been executed against the code, not written
from memory. Where a number appears it was measured on the run described.
Where something is not built, it says so.

---

## 0. What CommsEv is, in one paragraph

A command-and-control resilience testbed for multi-agent robotic fleets under
jamming and GNSS denial. Its claim is one causal chain that most tools break
in half:

> **RF physics → link state → command authority → agent behaviour → position
> knowledge → mission outcome**

The programme is not "add features". It is one question asked of progressively
richer payloads: **what does contested spectrum actually cost you?** A link
model with no fleet on it answers nothing; a fleet simulator with a link
quality slider answers nothing either. The whole point is that the physics
reaches the mission score without anybody hand-wiring it.

The contribution worth defending in a viva is the **separation of authority
from routing**. `authority` (centralized / decentralized / hierarchical) is who
*decides*. `routing` (star / mesh / tiered) is how the message *travels*. They
are usually conflated. Here they are independent axes, both consuming one
`command_parents()` tree, and you can sweep either against the other.

---

## 1. The ten-minute demo

Run it in this order. Each step is checked below with what you should see.

### 1.1 Open it

`console/app.py` — or the desktop shortcut. The menu bar reads
**File · Simulation · Scene · Agent**. Scene and Agent are deliberate stubs
that say "NOT BUILT YET" and what they will be; Simulation is the whole
application today.

### 1.2 Setup — compose a world

Left panel, **Setup** tab. Note that *every other tab is greyed out*. That is
deliberate: you cannot look at results that do not exist yet.

1. **Scene** → `lab_box`. (8 × 8 m room. `maze` is the interesting one —
   20 × 20 m with eight interior walls of mixed material.)
2. **Blue fleet** → `3_roboracer`. A spawn dialog opens: it asks where the
   fleet starts, what shape it holds and at what spacing. Accept the defaults.
3. **Red fleet** → `red_jammer`. One jammer.
4. **Add point** three times, and drag them into a triangle. Call them
   P1 P2 P3 (they are named automatically in order).

> **Scenes ship no points.** A scene is the *world*; where you send a fleet
> inside it is a decision about *this run*. Verified: `lab_box` defines
> none. `default_run.yaml` carries P1–P4 because a default run has to be
> runnable with nothing set up.

### 1.3 Play, and give it a mission

Press **▶**. Then in the white (umpire) terminal at the bottom:

```
SETMISSION patrol to P1 P2 P3 laps 2
blue launch
```

**What you should see** — verified end to end on `default_run.yaml`:

```
SETMISSION: mission 'patrol' set (3 agents tasked)
LAUNCH: blue (3 agents)
...
all complete at t=22.4 s
mission score: tasked 3, complete 3, pass_frac 1.0
```

and in the blue cell, `mission 100% (3/3) — MISSION COMPLETE`.

Watch the corners. The formation **pivots** — it rotates about its centre
rather than sliding sideways and re-forming — so car1 starts on the left and
ends on the left. Measured: 180° of swing over the circuit, identity kept.

### 1.4 Now jam it — this is the headline result

Restart (**↻**), then:

```
SETMISSION patrol to P1 P2 P3 laps 2
JAM jam1 power 10 band 2400
red launch
blue launch
```

Then run it again with `networks: blue: authority: decentralized` (Setup →
Scenario → authority).

**Measured on `lab_box` + `3_roboracer` + `red_jammer`, patrol P1 P2 P3, 2 laps,
seed 1:**

| Jammer power | centralized | decentralized | hierarchical |
|---|---|---|---|
| none | **3/3** | **3/3** | **3/3** |
| 0 dBm | 0/3 | **3/3** | 0/3 |
| 10 dBm | 0/3 (2 awaiting orders) | **3/3** | 0/3 (2 awaiting orders) |
| 20 dBm | 0/3 (3 awaiting orders) | 0/3 (0 awaiting orders) | 0/3 |
| 30 dBm | 0/3 | 0/3 | 0/3 |

That table is the whole project in one screen. A centralized fleet stops
because its *orders* stop arriving — `AWAITING ORDERS` is 2 or 3. A
decentralized fleet keeps going through the same jamming, because nothing has
to arrive for it to act.

**And note the honest wrinkle at 20 dBm:** decentralized fails with
`awaiting_orders = 0`. It did not lose command — it lost something else.
Suspected mechanism: the fleet centre is built from *peer reports* that travel
the jammed link, so formation keeping degrades before command does. **This is
unverified — do not quote it as a finding.** It is written up in
`progress/objectives.md` as an experiment worth running.

### 1.5 Show the spectrum

Load `maze` (it has walls). Toolbar:

- **▦ spectrum** — received power in dBm at every point on one band, drawn as
  a field with contours every 10 dB and the receiver-sensitivity line picked
  out. The metal wall on the right casts a hard radio shadow.
- **The three sliders — x, y, z.** These are the CAD-style **section planes**.
  Top view reads Z, Front reads Y, Side reads X, and all three hold their
  position when you switch view. Drag Z from 0.2 m to 2.5 m and watch the
  1.2 m brick partition's shadow *disappear* — measured 75 dB of spread on
  the floor against 47 dB at 2.5 m. That difference is the entire argument for
  putting a relay on a mast.
- **⧖ wavefront** — each transmitter's reach as equal-power rings, blue for
  friendly, red for hostile, dashed for a jammer. Shaded ground is where the
  hostile signal is the louder one.
- **✂ cut away** — opens the scene on the three planes so you can see inside a
  building, in *any* view including ISO. **Purely visual**: it changes drawing
  opacity and whether a wall is painted, and touches nothing else. No agent
  moves, no file is written, a run in progress does not notice.

### 1.6 One experiment, from Setup

**Experiment** tab → **Run an experiment…** at the top.

Axes: `authority` × `jam_rel_db` (with `routing` pinned to `star`). Press run.
It sweeps the grid headlessly, then draws the result. Click a row in the
results table and **that curve lights orange** while every other line drops to
the plain colour of its authority — four things in the key whatever the number
of curves. Double-click to watch that cell replay live.

Hover any column heading for what the metric means.

---

## 2. Every command in the sandbox

Three terminals: **white** (umpire — anything, plus the shell), **blue**, and
**red**. A cell may only act on its own side; a refusal prints why and is not
sent.

### 2.1 Mission and command

| Command | What it does | Cell |
|---|---|---|
| `<who> launch` | Arm — objectives start acting. `who` is a network (`blue`) or an agent id. A network-wide launch skips ground stations. | own side |
| `<who> halt` | Un-arm — freezes at the current pose. | own side |
| `SETMISSION <name>` | Apply `missions/<name>.yaml`, gated by command authority — an unreachable agent is *not* retasked. Titles the run. | blue |
| `SETMISSION <name> to <P1> <P2> …` | …and say where its goals are. This is what lets one mission file run on any scene. | blue |
| `SETMISSION <name> to <P1> <P2> laps <n>` | …and how many times round. | blue |
| `SETPLAN <who> <P1> <P2> … [laps <n>]` | The mission typed directly, no file. `who` is `all`, a network, or an agent id. | blue |
| `REOBJECTIVE <agent> <objective> <args…>` | Retask one agent live. | own side |

**Objectives available to `REOBJECTIVE`** (verified in `parse_retask`):

```
advance                 forward until something stops you
advance FAR             the fleet's CENTRE lands on point FAR
advance (90,0,0)        ...or on that coordinate
pursue <agent>          (also spelled 'pursuit')
shuttle A B             between two points
shuttle (-3,3,0) (3,3,0)    literal coordinates, or a mix
wall_follow [left|right]    (also 'wall')
orbit [radius]
script <file>
```

Example: `REOBJECTIVE car1 pursue car3`

### 2.2 Electromagnetic

| Command | What it does | Cell |
|---|---|---|
| `JAM <id> [band <MHz>] [power <dBm>]` | Tune a running jammer. One line so band and power can never disagree — two separate commands left a window where the jammer sat on the new band at the old power. On/off is `launch`/`halt`. | red |
| `TXPOWER <id\|network\|all> <dBm>` | The friendly twin of `JAM`. Range −30 to 60 dBm; a bad line applies nothing. `all` needs the white cell. | own side |
| `LISTEN <MHz>` | Tune this cell's receiver and print every transmission on that band your own agents could actually hear. `LISTEN 2400` for the command net, `LISTEN 1575.42` for GPS L1. | red |
| `LISTEN off` | Receiver off. | red |
| `LISTEN` | What am I tuned to? | red |

`JAM` and `TXPOWER` both write one line into the run's retask spool, which the
running sim reads on its next tick. That is the same path the Console's own
double-click editor uses — **Contested** tab, double-click an Emitter or a
Radio.

### 2.3 View

| Command | What it does |
|---|---|
| `CUT` | Report the three section planes and whether the scene is cut. |
| `CUT x\|y\|z <metres>` | Move one plane. Refused outside the scene, with the range. |
| `CUT on` / `CUT off` | Open or close the scene on them. |

`CUT` is the **only** command that does not reach the model. It moves the
planes the viewport draws on and nothing else, which is why it is not written
to the retask spool. Every cell may use it — looking at something is not an
action against a side.

### 2.4 Shell

Anything unrecognised goes to bash, with ROS 2 and the workspace already
sourced. `cd` works; interactive tools (`sudo`, `vim`) need a real terminal.

---

## 3. What the application can do today

### The model (`tools/stub_telemetry.py`)

**Radio.** Log-distance path loss at the scene's own exponent, a declared
noise floor, and every same-band emitter summed into the denominator. SINR is
the single currency: distance, walls and jamming all reduce to it, it becomes
packet delivery ratio, and that becomes link state. A jammer is not a special
case — it is another term in the denominator, which is what lets a jammer be
an ordinary agent.

**Adjacent-channel leakage.** `aci_mu()` — an off-band emitter leaks in rather
than vanishing, so tuning away from a jammer *fades* it. At 2.4 → 5.8 GHz that
is essentially zero, which makes the second band a genuine escape.

**Walls.** A wall is a line segment with a material, a height and a thickness,
and it is consumed by radio, lidar and collision alike. A signal takes the
better of two paths: **through** (per-material penetration loss) or **around**
(knife-edge diffraction). Taking the minimum is the physical statement, and it
is what stops a metal wall producing an absurd 200 dB link when the two agents
can plainly see each other round the corner. Height is interpolated along the
path, so a link that climbs over a half-height wall pays nothing for it.

**Spectrum field.** Received power in dBm at every point on one plane —
horizontal at a height, or a vertical section at an x or a y. A scalar field,
which is why it can be contoured at all. It is *not* SINR: SINR needs a
transmitter and a receiver, so it belongs to a link, not to a place. Read the
field as coverage, not as quality.

**Wavefront and contested ground.** Each emitter's reach as equal-power rings
at absolute dBm levels, so a blue ring and a red ring at the same level mean
the same thing. Rings are traced along 96 bearings paying the same wall loss,
so they dent behind a wall and reach round its end. `advantage_field()` is
blue power minus red power in dB at every point — where it goes negative, the
loudest thing on the band belongs to the other side.

**Command.** One `command_parents()` tree, read by both routing and authority.
`command_chain()` walks to the coordinator, which is the right denominator for
command load — so a three-deep chain reads **3 / 2 / 1**, not 1 / 1 / 1.
Reachability is scored on the *pre-step* link states, which is the honest
one-tick lag: an agent acts on the link it last had.

**Formations.** Slots captured once per side, rotated by the leg's bearing,
pivoting at ω = v/r. A leg completes when the *formation* arrives, not when one
vehicle does. A shape that will not fit **shifts** rather than being clamped
(clamping collapsed two slots onto one coordinate and froze the mission at 0%).

**Navigation.** Position *belief* drifts separately from truth. GNSS denial is
a jammer on 1575.42 MHz; an agent with a lidar or vision aiding sensor keeps a
fix. The fleet centre is built from peer reports that travel the jammed link,
so a fleet can be confidently wrong about where it is.

**Sensors.** Hokuyo UST-10LX (270°, 10 m) and Intel RealSense D435i
(87° × 58°, 0.28–3 m) as genuinely different instruments — the depth camera is
published as a horizontal slice of the depth image and says so. Per-material
reflectance changes effective range.

### The Console (`console/app.py`)

Setup (scene, fleets, points, formation, authority, routing) · live 2D/3D
viewport with TOP/FRONT/SIDE/ISO · three section planes and cut-away ·
spectrum field with contours · wavefront rings and contested ground · command
layer (authority arrows, rank glyphs, per-link load) · per-sensor tabs with
polar plots and a depth strip · three cell terminals · Contested tab with live
emitters and radios, double-click to tune · experiment builder with swept axes
· results table with hover definitions · plots where colour is authority, dash
is routing, and the clicked row lights orange · Copy findings.

### Headless (`tools/sweep.py`)

```
python3 tools/sweep.py experiments/intercept.yaml
python3 tools/sweep.py experiments/intercept.yaml --jobs 8 --out runs/
python3 tools/sweep.py experiments/intercept.yaml --replay <cell-key>
```

Parallel across cores, writes CSV, and `--replay` re-runs one cell and writes
its frames as JSONL so a single interesting cell can be watched.

### Assets

**Scenes** `lab_box` · `corridor_200m` · `open_field` · `maze`
**Agents** `roboracer` (+ `_lidar` `_depth` `_full` `_dualband`) ·
`ground_station` · `jam_single` · `jam_dual` · `listener`
**Fleets** `3_roboracer` · `3_roboracer_no_lidar` · `red_jammer` · `red_comms`
· `red_gnss` · `red_both` · `custom_blue` · `custom_red`
**Missions** `patrol` · `advance` · `forward` · `shuttle` · `test` ·
`wall_follow.py` · `example_pursuit.py` · `return_on_link_loss.py`
**Experiments** `intercept` · `penetration` · (`authority_vs_jamming`, to be
deleted — see below)

### ROS 2

A package under `ros2/src` that publishes the same frames on real topics,
including per-sensor scans and the camera's `CameraInfo`. **Written but never
run on a live graph.** Treat it as unproven.

---

## 4. The rule that governs everything

From `CLAUDE.md`:

> Every physical quantity is `{value, unit, source}`. Never invent a number
> that looks researched. Comparative claims only.

A blank `source` is a *declared free parameter*, not a disguised guess. That
distinction is the reason the work is defensible, and it is the single habit
to keep.

**Honest total (live files, Sep 2026): 35 of 57 declared quantities carry a
source. 22 do not.** Every unsourced one is declared — but a result that turns
on any of them must say so.

---

## 5. References, and where each one went into the app

### In use — the number is running in the code

| Source | What it gave | Where it lives |
|---|---|---|
| **Friis**, independently matched by Zhou et al. 2020 (arXiv:2008.08212) | Path-loss form `20lg(f) + 20lg(d) + 32.44` | `rf_link()`, `spectrum_field()`, `wavefront_rings()` |
| **Papadopoulos & Misailidis**, *On Differential Drive Robot Odometry*, ECC 2007, Table I | Unaided wheeled dead-reckoning drift **0.49 %** of distance (Pioneer 3-DX, uncalibrated, worst case) | `drift_rates` — what GNSS denial costs a ground vehicle |
| **Robert Wilson**, *Propagation Losses Through Common Building Materials*, Magis Networks E10589, 2002, Table 3 | Per-material 2.4 GHz penetration: drywall 0.49 · glass 0.50 · fir 2.79 · brick 4.44 · cinder block 6.71 dB | `MATERIALS[…]["rf_db"]`, used by `wall_excess_db()` |
| **ITU-R P.526-16** (11/2025) §4.1, eq (26) and (31) | Knife-edge diffraction `J(v)`, valid v > −0.78 | `knife_edge_db()` — the "around" path |
| **Zhou et al. 2020** | Structure of the adjacent-channel correlation μ(Δf) | `aci_mu()` |
| **Hokuyo UST-10LX spec** §2-2, §4 | 10 m against white paper, 4 m at 10 % diffuse | `effective_range()` interpolates between those anchors |
| **Intel RealSense D435i spec** | 87° × 58°, 0.28–3 m, < 2 % at 2 m | the depth-camera agent and its cone |
| **AJP-3 Ed D** §3.8, §3.11, §3.29 | Mission command — a commander acts on intent when the link is gone | `on_link_loss: intent`, and decision **D5** |
| **AJP-3 Ed D** §C.21 | Interference framed as a *command* problem, not an RF one | the framing of the whole project |
| **ArduPilot** Copter, GPS Failsafe | ~10 s usable window before divergence | GNSS outage behaviour |
| Standard 4/3-earth radio horizon | `4120(√h₁+√h₂)` | radio horizon check |
| **GPS L1** | 1575.42 MHz | the GNSS band |

### Sourced and deliberately *not* used — read before touching these

| Source | What it says | Why not used |
|---|---|---|
| **LOAM** (Zhang & Singh, RSS 2014) Table I | Lidar scan-matching drift **0.9 %** over 58 m, open loop | *Worse* than the 0.49 % wheel figure. What a lidar buys is error that **stops growing**, not error that grows slower — that is decision **D1** |
| **Ding et al.**, arXiv:1503.04251v3, §VII | NLoS is a **steeper exponent** (α_L 2.09 → α_NL 3.75), not just extra dB | That is the *outdoor stochastic* form, where LoS is a probability. We compute LoS **geometrically** from real walls, which is better information — applying both would double-count the same obstruction |
| Same paper | Coverage first improves then **degrades** as a network densifies, because the interference becomes LoS too | A warning about our own routing axis: **a mesh in a maze may be worse than a star.** Never tested; the model can now express it |
| **Fahad & Bulut**, VCU, Table II | An NLoS link sharing a channel scored 68 %; on a clear channel **89 %** | The metric is activity-recognition accuracy, not PDR, so it cannot be lifted. Supports a principle: the agent already round a corner is the one congestion hurts most |
| **ITU-R P.2040-3** §2.2.2 eq (43b), Table 3 | The standards route to computed dB per wall from permittivity, thickness and incidence | Worth doing if a result ever turns on the exact number |
| **ITU-R P.1238-9** Table 2 | Indoor distance power-loss coefficient N = 28/30 | N averages walls *into* a whole-path exponent over tens of metres — wrong tool for a 1–5 m maze. Also: it contains **no** per-material table, it defers to P.2040 |
| **Garnaev et al.**, IEEE Comms Letters 2021, Prop. 6(a) | Against a jammer the transmitter's equilibrium power is **lower**, and its payoff no worse | The answer to "is it simply speaking louder?" — **no** |
| **Altman et al.**, NET-COOP 2007, Thm 5–7 | With transmission cost the game is non-zero-sum with a unique non-maxed equilibrium | Same conclusion, different route |
| **Priyadarshani et al.**, IEEE OJ-COMS 2025 §III-A | Low power makes you **undiscoverable**; raising it helps only after the jammer is found | Both horns in one place. Already in the project folder |
| **Bash, Goeckel & Towsley**, IEEE JSAC 2013 | The **square root law** — covert transmission is fundamentally power-limited | Gives the shape of the trade-off, not a detection range |
| **NAWCWD TP 8347** §4-6, §5-2, §4-7 | EMCON power-density ceiling; receiver sensitivity equation; jammer burn-through | §4-7 is the **two-way radar** equation — do not port its coefficients to a comms link |
| **Li, Hou & Sha** (LMST), INFOCOM 2003 · **CBTC**, INFOCOM 2001 · **COMPOW** | Implementable minimum-power-per-neighbour algorithms; the α ≤ 5π/6 threshold | Indoor multipath breaks the RSS → distance inversion; CBTC needs angle-of-arrival a commodity module lacks |
| **Gupta & Kumar**, IEEE TIT 2000 | Θ(1/√(n log n)) scaling | **Correction to a common misreading:** it makes *no* prescription that minimum power is better. Cite for the scaling law only |
| **Xu et al.**, ACM MobiHoc 2005 | A **reactive** jammer must contain a receiver. And: signal strength alone cannot detect one — **PDR is the statistic** | The second half matters most: **nothing in CommsEv detects jamming.** Blue suffers it without noticing |
| **ITU-R Spectrum Monitoring Supplement** 2008, §3.2.3.3 | Monitoring-station antenna gain 10–12 dBi | The sourced asymmetry against a robot's 0–2 dBi whip. Our listener has **no gain**, so every interception figure is a lower bound |
| **TI CC2420** datasheet SWRS041c | Sensitivity −90/−95 dBm; **8 discrete power steps over 24 dB** | Power control is not a continuous variable on real hardware |
| **US Patent 7,099,369** (Karlsson) | A follower jammer is receiver + DF + transmitter in one box | Evidence of a claimed architecture, never of fielded performance |
| **Bergman**, Linköping Diss. 579, 1999, Ch. 7 | TRN accuracy has a **CRLB in terms of the terrain gradient**. Flat terrain carries *no* position information | For a future contoured scene. Airborne formulation — structure transfers, numbers do not |
| **Lee et al.**, Remote Sensing 12(9):1396 · **Jarraya et al.**, Sat. Nav. 6:9 · **Golden**, SPIE 0238 | TERCOM/SITAN/DSMAC as established GNSS-denied methods; degradation in featureless terrain | Qualitative; no metre figures |
| **Choi et al.**, Consensus-Based Decentralized Auctions (CBBA) | Decentralised task allocation with real convergence traffic | Our decentralized agents **do not negotiate** — a control condition, not a claim |

### Declared free — no source found, and that is stated

Path-loss exponent (2.8 indoor / 2.2 open) · noise floor −95 dBm · receiver
sensitivity −85 dBm · **the PDR-vs-SINR logistic curve** (shape right, values
not a real radio) · GNSS denial threshold −120 dBm · drift heading random-walk
σ · adjacent-channel rejection 30 dB · channel bandwidth 20 MHz · contention
audibility threshold · metal wall opacity (200 dB, declared so that diffraction
round the end is the only way through) · **lidar reflectance of interior
finishes** (the mapping from reflectance to range *is* measured; the input to
it is a guess).

---

## 6. Known weaknesses — stated, not hidden

- **Contention counts agents, not traffic.** Every routing advantage is an
  upper bound.
- **Decentralized agents do not negotiate.** A control condition.
- **No loop closure**, so aided error grows without bound — decision **D1**.
- **Nothing detects jamming.** Blue never notices; it only suffers.
- **The intercept listener is omni, no gain, no better than the intended
  receiver.** Every interception figure is a lower bound — and in a 200 m
  corridor with kilometre-range radios it hears everything, which is a real
  result but a trivial one.
- **Formation cornering.** Spacing holds to under a millimetre on a straight
  leg and deforms to **2.7 m off a 2.00 m nominal** through a corner. Cause:
  the formation reference runs *ahead* of the fleet, so the leader carries a
  4.9 m standing error against the trailer's 2.5 m, pace scales with those
  errors, and the shape closes up. The fix is a virtual leader — a design
  decision, open.
- **The formation's turn rate is derived, not measured** (ω = v/r).
- The sim publishes a **horizontal slice** of the depth image, not a volume.
- **Jammer geometry is not swept.**
- `experiments/authority_vs_jamming.yaml` is **to be deleted** — it crossed
  authority × routing × power over a mission with nothing in the way, so every
  cell measured what `penetration.yaml` measures with fewer cells.

---

## 7. Open decisions — these need a person, not a commit

| # | Decision | Why it needs you |
|---|---|---|
| **D1** | **Bounded error vs a drift rate** — model loop closure so aided error stops growing? | LOAM measures 0.9 %/distance open loop, *worse* than 0.49 % wheel odometry. What a lidar buys is error that stops growing. A modelling claim to make |
| **D2** | **Deception** — should a spoofed order be *accepted*? | Red demonstrably hears the orders. Accepting a forged one is a claim about the radio having no authentication |
| **D3** | **`missions/test.yaml`** — keep the welded literal-coordinate benchmark? | The one deliberate exception to "missions name no geometry" |
| **D4** | **A formation that will not fit** — it SHIFTS. Right, or should a fleet fan out? | It changes where "arrived" is, and therefore what a mission scores |
| **D5** | **A leader on `intent` still commands its squad.** Doctrinally right (AJP-3 3.8/3.11) but a modelling claim | It decides how much a hierarchy is worth under jamming |
| **D6** | **The virtual leader** — should a formation's reference advance only at the pace the fleet can hold? | Fixes cornering; also changes what "arrived" means, so it touches D4 |

---

## 8. Where to start, if you are taking this over

1. **Run the tests.** `python3 tests/test_all.py` → 522 passing. They are
   written as sentences; read the failures, not the tracebacks.
2. **Read `progress/objectives.md`.** It is the durable list, carried across
   sessions. Nothing is deleted until it is genuinely done.
3. **Read `docs/vocabulary.md`.** Scene / fleet / mission are three layers and
   the distinction is load-bearing.
4. **Then pick one of:** the wall experiment (does a richer sensor fit buy you
   anything measurable?), frequency agility (EVADE / FOLLOWED), or the mesh-in-
   a-maze test the Ding paper suggests.

**The standing constraint that gates everything:** open-sourcing must be
cleared **in writing** with Cheng and Loughborough IP before anything is
published. The repo stays private until then. This is irreversible.

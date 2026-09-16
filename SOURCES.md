# Sourcing register

Every physical parameter in CommsEv carries a source. This file tracks what
still needs one.

**Rules.** No number enters the codebase without a source. A source is a
datasheet, a paper, or a measurement we took ourselves and can describe. Where
no good source exists, say so and make the parameter explicitly free rather than
inventing a value that looks researched.

Status: `TODO` · `FOUND` · `MEASURED` · `NO SOURCE` (declared free parameter)

### Open, 2026-09: what a lidar actually buys is not a lower drift rate

Searched for the aided figure and found something more useful than a number:
**the measured evidence does not support the claim that lidar aiding reduces
drift per metre.** LOAM measures 0.9% of distance in an indoor corridor with no
loop closure — *worse* than the 0.49% measured for plain wheel odometry.

That is not a contradiction, it is the wrong question. What a lidar or a depth
camera buys is **bounded** error: return to a place you have already mapped and
the accumulated error is corrected, which is exactly why Intel's T265 figure is
quoted **closed loop**. Error that stops growing is a different property from
error that grows more slowly, and this model has only the second.

So the honest state is: both aided constants are held at the unaided rate ("an
aiding sensor is at least not harmful"), and the real fix is **structural** —
model loop closure so aided error is bounded in a mapped space — not a better
constant. That is a modelling decision, flagged rather than taken.

### Correction, 2026-09: the unaided drift rate was invented

It was `0.04` — 4% of distance travelled — attributed to a MEMS inertial
figure for an autopilot. It is an **order of magnitude worse than anything
measured** for the vehicle class actually being simulated, and the consequence
was not subtle: on any scene declaring GNSS denied (`lab_box`, indoors, no sky
view) a fleet accumulated a quarter of a metre of error over a 6 m lane, so
every mission failed on drift within seconds and the scene was unusable.

Replaced with a measured figure from a paper on exactly this vehicle class.
The result changed a headline finding: under GNSS denial on the 200 m corridor
the fleet now scores a **partial** pass — some vehicles report a false arrival
and some do not — where the invented rate failed every vehicle every time.
That looked decisive and was an artefact.


## Datasheet lookups

| Parameter | Status | Source |
| --- | --- | --- |
| Lidar range, FOV, scan rate, accuracy (UST-10LX) | TODO | Datasheet in project files |
| Radio tx power, sensitivity, data rate | TODO | Depends on module choice — pick hardware first |
| Rover mass, wheelbase, CoG, steering limits | TODO | Traxxas spec + our own measurements |
| Rover dimensions | MEASURED | Car #1, 2026-07-21 |

## Needs literature

| Parameter | Status | Notes |
| --- | --- | --- |
| Lidar noise model beyond datasheet accuracy | TODO | Hokuyo characterisation papers exist |
| IMU bias instability, random walk | TODO | Allan variance characterisation |
| Indoor path loss exponent, confined room | TODO | Matters most for the 8 m box |
| Multipath fading model, K-factor, walled arena | TODO | Depends on wall material |
| Outdoor propagation, short ground strip | TODO | Ground reflection is not negligible at 80 m |
| Background spectrum occupancy per band | TODO | Also replaceable by our own field recordings |
| Quadcopter thrust and drag coefficients | TODO | Rotor characterisation, or a thrust stand |
| GNSS receiver behaviour under spoofing | TODO | Well covered in GNSS security literature |
| Estimator innovation gate thresholds | TODO | ArduPilot defaults are in source; rationale is in the literature |

## Probably no clean source

| Parameter | Status | Notes |
| --- | --- | --- |
| Fiber tether mass/m, drag, bend radius, spool | NO SOURCE | Manufacturer data plus our own measurement. Model should say so. |
| Emissions detectability, time-to-geolocate | NO SOURCE | Open literature thins out fast. Model parametrically, declare free. |

---

## Model constants (added Aug–Sep 2026)

The RF / jamming / navigation model. Status per the rules above.

| Parameter | Value | Status | Source |
| --- | --- | --- | --- |
| Path-loss form `20lg(f)+20lg(d)+32.44` | — | FOUND | Friis; independently matched by Zhou et al. 2020 (arXiv:2008.08212) |
| Path-loss exponent (indoor) | 2.8 | NO SOURCE | Defensible indoor-multipath default; scene declares it |
| Path-loss exponent (open field) | 2.2 | NO SOURCE | Near-free-space default; scene declares it |
| Noise floor | −95 dBm | NO SOURCE | Defensible default; measure with an SDR to upgrade |
| Receiver sensitivity | −85 dBm | NO SOURCE | Commodity 2.4 GHz figure; module datasheet would upgrade |
| PDR-vs-SINR curve | logistic, k=0.8 | NO SOURCE | **Stand-in for a modulation/coding curve. Shape is right, values are not a real radio.** |
| Radio horizon `4120(√h₁+√h₂)` | — | FOUND | Standard 4/3-earth radio horizon |
| GNSS band | 1575.42 MHz | FOUND | GPS L1, standard |
| GNSS denial threshold | −120 dBm | NO SOURCE | GNSS arrives ~−128 dBm ("weak and easily jammed", Critical Analysis of Spoofing & Jamming); −120 is a declared free parameter |
| Dead-reckoning drift, unaided, **wheeled** | 0.49% of distance | **MEASURED** | Papadopoulos & Misailidis, *On Differential Drive Robot Odometry with Application to Path Planning*, ECC 2007, Table I: Pioneer 3-DX over ~125 m paths, uncalibrated error 16.1–60.9 cm = 0.13–0.49% of distance. Worst case used — an uncalibrated vehicle is the honest default |
| LiDAR scan-matching drift, open loop | 0.9% of distance | **MEASURED, not yet used** | LOAM (Zhang & Singh, *LiDAR Odometry and Mapping in Real-time*, RSS 2014) Table I: **0.9% over 58 m** and 1.1% over 46 m in an indoor corridor, 2.3–2.8% outdoors, explicitly **without loop closure** |
| Visual-inertial drift, closed loop | <1% | **MEASURED, wrong quantity** | Intel specify "<1% drift" for the T265, but that is the error on *returning to a place already seen* — a closed-loop figure, not open-loop drift during an outage. Not substitutable |
| Dead-reckoning drift, fused wheel + lidar | — | **OPEN** | The case the model actually needs. Both single-modality figures above bracket it (0.49% wheels alone, 0.9% lidar alone) and fusion should beat both, but no measured fused figure found. `DRIFT_RATE_LIDAR` and `DRIFT_RATE_DEPTHCAM` are held at the unaided rate meanwhile |
| Depth camera D435i geometry | 87°×58°, 0.28–3 m, <2% at 2 m | **FOUND** | Intel RealSense D435i product specification: min depth ~28 cm, ideal range 0.3–3 m, depth FOV 87°×58°, up to 1280×720 at up to 90 fps, RGB 1920×1080 at 30 fps, accuracy <2% at 2 m |
| Dead-reckoning drift, **airborne** | — | **OPEN** | A quadcopter has no wheel odometry and drifts on inertial integration. A different number entirely; must not be assumed equal to the wheeled figure. Blocks the UAV transition |
| GNSS usable window before divergence | ~10 s | FOUND | ArduPilot Copter, GPS Failsafe & Glitch Protection |
| Drift heading random-walk σ | 0.15 rad/tick | NO SOURCE | Free parameter; sets how error meanders, not how fast it grows |
| ACI correlation coefficient μ(Δf) | — | FOUND (structure) | Zhou et al. 2020 — properties (μ=1 at Δf=0, →0 far apart, symmetric, measurable) |
| Adjacent-channel rejection | 30 dB/channel | NO SOURCE | Commodity receiver figure; declared free parameter |
| Channel bandwidth | 20 MHz | NO SOURCE | Nominal |
| Contention airtime share `1/(n+1)` | — | FOUND | Channel-sharing arithmetic, not a fitted constant |
| Contention deadline model `1−exp(−deadline/delay)` | — | FOUND | Standard M/M/1 exponential service assumption; deadline comes from the network's own declared QoS |
| Contention audibility threshold | −85 dBm | NO SOURCE | Same as receiver sensitivity; declared free parameter |
| Vehicle max accel (default) | 2.0 m/s² | NO SOURCE | Nominal for a 1:10 car; **measure the real cars** |
| Vehicle min turn radius (default) | 0.6 m | NO SOURCE | Nominal; **measure the real cars** |
| LoS / NLoS excess path loss | 3 dB / 23 dB | FOUND, NOT YET USED | Zhou et al. 2020 — we do not model an LoS/NLoS split yet |

---

## Power control: is it simply speaking louder? (searched 2026-09-10)

Asked because a fleet that could send at the **minimum power needed to reach
the next agent** might slip under local noise jamming — and because the same
choice decides whether the other side can hear it (see
`experiments/intercept.yaml`). **Nothing has been implemented from this. It is
a reading list with the findings stated, so the decision is made on the
literature rather than on the intuition.**

**The short answer: no, and the literature is fairly clear about it.** Raising
transmit power is not the equilibrium strategy against a responsive jammer, and
it costs you twice — the jammer gains from power symmetrically, and a louder
transmitter is a more interceptable one.

| Claim available to us | Source | What it actually shows | Caveat |
| --- | --- | --- | --- |
| Against jammers the transmitter's equilibrium power is **lower**, not higher, when it commits first — and its payoff is no worse | Garnaev, Petropulu, Trappe, Poor, *A Multi-Jammer Power Control Game*, IEEE Communications Letters 25(9):3031–3035, 2021. Proposition 6(a), `P_S ≤ P_N`; §V | The cleanest statement that escalation is not the winning move: the strategic structure (who moves first) dominates raw power | SINR utility with an explicit linear transmission-cost term, single link. The conclusion depends on that cost term existing |
| With transmission costs the jamming game is non-zero-sum with a unique equilibrium in which the transmitter does **not** max out power | Altman, Avrachenkov, Garnaev, *A Jamming Game in Wireless Networks with Transmission Cost*, NET-COOP 2007, LNCS 4465, pp. 1–12. Theorem 7 (uniqueness), Theorems 5–6 (closed form) | Sets `c_t = c_n = 0` and it collapses to the classical zero-sum saddle point, so the cost term is what creates the result | Allocation across parallel channels, so it argues for spreading and channel choice as much as for power level |
| Low transmit power is itself a defence — it makes the transmitter **undiscoverable**; raising power helps only *after* the jammer has been detected | Priyadarshani, Park, Ata, Alouini, *Jamming Intrusions in Extreme Bandwidth Communication*, IEEE OJ-COMS vol. 6, 2025, §III-A "Regulated Transmit Power" | States both horns of the question in one place. **Already in the project** as `Jamming_Intrusions_in_Extreme_Bandwidth_Communication.pdf` | mmWave/THz/FSO/VLC, where power control pairs with beamforming — the paper says power regulation "works better" there for that reason. Does not transfer cleanly to omnidirectional 2.4 GHz |
| More jammer power is unambiguously good for the **jammer**: it pushes burn-through closer | NAWCWD TP 8347, *Electronic Warfare and Radar Systems Engineering Handbook*, 4th ed. 2013, §4-7 | The escalation is symmetric — both sides gain from power, which is why it is not a strategy | **Two-way monostatic radar equation** (note `20 log R` and the `σ` term), not a comms link. Do not port the coefficients. For a one-way comms J/S the right source is Poisel, *Modern Communications Jamming*, 2nd ed., Ch. 8 — **not yet read** |

**Minimum-power-to-a-neighbour, as an implementable algorithm**

| Claim | Source | Caveat |
| --- | --- | --- |
| A node can set **per-neighbour** transmit power from measured received signal strength and still provably preserve connectivity, node degree ≤ 6 | Li, Hou, Sha, *Design and Analysis of an MST-Based Topology Control Algorithm* (LMST), IEEE INFOCOM 2003. Measured mean degree 2.06 (LMST) vs 2.97 (CBTC) over 100 nodes | Assumes monotone distance-based path loss. Indoor 2.4 GHz multipath breaks the RSS → distance → power inversion, so the mechanism transfers and the numbers do not |
| Transmitting at the lowest **common** power that keeps the network connected raises capacity, extends battery life and cuts MAC contention | Narayanaswamy, Kawadia, Sreenivas, Kumar, *Power Control in Ad-Hoc Networks* (COMPOW) | Widely cited as European Wireless 2002 pp. 156–162, **venue unverified from the paper itself**. One global power level, not per-link |
| Minimum power such that every cone of angle α ≤ 5π/6 contains a reachable neighbour preserves connectivity; α > 5π/6 does not | Li, Halpern, Bahl, Wang, Wattenhofer, *Cone-Based Distributed Topology Control* (CBTC), IEEE INFOCOM 2001 | Needs angle-of-arrival at each node, which a commodity module does not have. Cite for the existence of a proved threshold, not as something our fleet could run |
| Per-node throughput scales Θ(1/√(n log n)) under a common power; the range/interference trade-off is quadratic in range | Gupta & Kumar, *The Capacity of Wireless Networks*, IEEE Trans. Inf. Theory 46(2):388–404, 2000 | **Correction to a common misreading:** the paper makes **no** prescription that minimum power is better. Cite it for the scaling law and the footprint-vs-hops trade-off only. Asymptotic in n; says nothing about ten robots in a room |

**Low probability of intercept, quantitatively**

| Claim | Source | Caveat |
| --- | --- | --- |
| Covert transmission is fundamentally power-limited: O(√n) bits in n channel uses, per-symbol power ~1/√n — the **square root law** | Bash, Goeckel, Towsley, *Limits of Reliable Communication with Low Probability of Detection on AWGN Channels*, IEEE JSAC 31(9), 2013 | AWGN, single warden, asymptotic, requires pre-shared secret. Gives the **shape** of the detection/communication trade-off, not a detection range for a room |
| EMCON doctrine converts "do not be heard" into an emitted power-density ceiling and back-solves the allowed transmit power: `PtGt = PD(4πR²)` | NAWCWD TP 8347, 4th ed. 2013, §4-6. Example limit −110 dBm/m² at 1 nmi → about −34 dBm at the antenna port | Naval/airborne, and the −110 figure is a **policy threshold**, not a physics constant. Use the equation; quote the number only as an example of the doctrine |

**Where this leaves the model.** The defensible comparative claim is *not*
"minimum power beats full power" — it is that **transmit power is a two-sided
choice**, and CommsEv is unusually well placed to show it because it already
models both sides: `rf_link` decides whether the order arrives and `intercept`
decides whether red hears it, off the same power. A `power_policy` axis
(`full` / `minimum-to-neighbour` / `escalate-when-jammed`) would put the
trade-off on one chart. Sourced enough to build; **not built.**

---

## Intercept and electronic-attack hardware (searched 2026-09-10)

For `agents/listener.yaml`, which currently assumes an omni antenna with no
gain and no receiver better than the intended one — deliberately, so every
interception figure is a lower bound.

| Quantity | Status | Source |
| --- | --- | --- |
| Fixed monitoring/DF station antenna gain | **FOUND** | ITU-R, *Spectrum Monitoring — Supplement to the Handbook*, 2008, §3.2.3.3 and §3.2.4.2: horizontally-polarised log-periodic "up to 12 dBi", VHF/UHF log-periodics "moderate gain (10 dBi typical)". Gives a sourced asymmetry against a robot's ~0–2 dBi whip |
| Intercept-receiver **noise figure** | **NO SOURCE** | Report ITU-R SM.2125-1 (2011) §2.3 **defines** monitoring-receiver noise figure and gives three measurement methods but publishes no typical value; vendor brochures omit it. Left blank rather than guessed |
| Commodity 2.4 GHz module, for comparison | **FOUND (partly)** | TI CC2420 datasheet SWRS041c: sensitivity −90 min / −95 typ dBm at 1% PER; **8 discrete output-power steps spanning 24 dB** (−24 to 0 dBm). The step granularity is a real modelling detail — power control is not a continuous variable. The datasheet states **no noise figure** either |
| Receiver sensitivity equation | **FOUND** | NAWCWD TP 8347 §5-2: `S_min = (S/N)_min · k·T₀·B · NF`, with `kT₀ = −174 dBm + 10log(BW Hz)` at 290 K. Lets NF and antenna gain be declared **free** while the ITU gain figures carry the sourced part |

**Can a jammer also be a receiver? Yes — and a reactive one must be.**

| Claim | Source |
| --- | --- |
| A **reactive** jammer "stays quiet until there is activity on the channel", so it necessarily contains a receiver. Four-model taxonomy: constant, deceptive, random, reactive | Xu, Trappe, Zhang, Wood, *The Feasibility of Launching and Detecting Jamming Attacks in Wireless Networks*, ACM MobiHoc 2005, pp. 46–57 |
| **Signal strength alone cannot detect a reactive or random jammer**; PDR is the statistic that separates jamming from congestion | Same paper. Directly relevant to how a blue fleet would ever *know* it is being jammed — a detection question we have not modelled |
| Reactive jamming is realisable on commodity SDR: FPGA on a USRP2, jamming bursts as short as **32 µs** for 802.15.4 | Wilhelm, Martinovic, Schmitt, Lenders, *WiFire: A Firewall for Wireless Networks*, SIGCOMM'11 demo, pp. 456–457 (companion to WiSec 2011 pp. 47–52). **Detect-to-jam latency not published in the demo** — read the WiSec paper before quoting a reaction time |
| A **follower jammer** is a receiver + direction-finder + transmitter in one unit: three spaced antennas → wideband receivers → FFT → Watson-Watt DF → jamming decision → transmit, jamming only bearings it selects | Karlsson, *Method and apparatus for surgical high speed follower jamming based on selectable target direction*, US Patent 7,099,369, Networkfab Corp., 2006. A patent is evidence of a claimed **architecture**, never of fielded performance |
| Follower jamming of frequency-hopping systems, and a **communications** J/S | Poisel, *Modern Communications Jamming Principles and Techniques*, 2nd ed., Artech House 2011, Ch. 8, 10, 11. **Table of contents verified, chapters not read** |

**What this means for the model.** `agents/listener.yaml` and
`agents/jam_single.yaml` are currently separate because the model has no reason
to combine them — a constant jammer does not need ears. The literature says a
*reactive* jammer does, and that a real follower jammer is one box. So the
honest next step is not to merge them but to add the capability that needs it:
a reactive emitter, which listens and then transmits. Sourced enough to build;
**not built.**

---

## Walls, line of sight and terrain (searched 2026-09-10, PARTLY IN USE)

Added when `scenes/maze.yaml` gave the model its first geometry inside a room.
Until then every link was line-of-sight by assumption, because nothing could
interrupt one.

| Quantity | Value | Status | Source |
| --- | --- | --- | --- |
| Wall penetration loss, 2.4 GHz, by material | drywall 0.49 dB · glass 0.50 dB · fir lumber 2.79 dB · dry red brick 4.44 dB · dry cinder block 6.71 dB | **MEASURED, IN USE** | Robert Wilson, *Propagation Losses Through Common Building Materials: 2.4 GHz vs 5 GHz*, Magis Networks report **E10589**, Aug 2002, **Table 3**. Anechoic chamber, two horns 16 ft apart, network analyser, time-domain gated. **Three caveats: measured at 2.3 GHz not 2.4; single thin SAMPLES at normal incidence, not built walls — a real stud partition runs several dB where the board alone runs half of one; and an industry report, not peer reviewed.** These are MATERIAL coefficients, not WALL coefficients |
| Metal wall penetration | opaque (200 dB) | **NO SOURCE — declared** | Wilson has no metal row and a conducting sheet does not usefully transmit. Declared opaque so the **diffraction path round the end is the only way through**, which is the correct physical picture and is what makes a metal wall interesting rather than fatal |
| Knife-edge diffraction loss | `J(v) = 6.9 + 20log₁₀(√((v−0.1)²+1) + v − 0.1)` dB | **FOUND, IN USE** | Recommendation **ITU-R P.526-16** (11/2025), *Propagation by diffraction*, **§4.1**, parameter v at **eq (26)**, loss at **eq (31)**, valid for **v > −0.78** (not −0.7). **P.526 does not use the phrase "Fresnel-Kirchhoff" — that is textbook terminology.** §4.1 is explicitly an *"extremely idealized"* single isolated perfectly-absorbing edge in free space; a maze corner is thick, finite, reflective and surrounded by multipath. Used as a **first-order softening of a binary LoS test**, which is far better than a cliff, and not as a claim of accuracy |
| Per-wall computed penetration from material properties | — | **FOUND, NOT USED** | Recommendation **ITU-R P.2040-3** (08/2023) **Table 3** (permittivity and conductivity coefficients for concrete, brick, plasterboard, wood, glass, metal and others) with the single-layer slab transmission coefficient at **§2.2.2 eq (43b)**. The standards route to a computed dB per wall as a function of material, thickness and incidence angle. Worth doing if a result ever turns on the exact number |
| Indoor distance power-loss coefficient N | 28 residential / 30 office at 2.4 GHz | FOUND, NOT USED | **ITU-R P.1238-9** (06/2017) **Table 2** (renumbered Table 4 in P.1238-13). **Checked explicitly: P.1238 contains NO per-material wall penetration table** — it defers to P.2040. And N is a whole-path exponent with walls *averaged into it*, calibrated over tens of metres, so it is the wrong tool for a 1–5 m maze where a specific wall sits between two specific robots |
| Measured building-material attenuation, broadband | — | FOUND, has a hole | **NISTIR 6055** (Stone, NIST, Oct 1997) measured 0.5–2 GHz and 3–8 GHz — **2.4 GHz was not measured.** Extrapolating from the 2 GHz edge is defensible if stated. Per-material frequency plots, no summary table |
| COST 231 multi-wall light/heavy wall values | — | **UNVERIFIED** | The primary COST 231 Final Report could not be retrieved. **Do not cite the L_w values** on secondary authority |
| Diffuse reflectance of interior finishes at ~905 nm | plasterboard 0.70 · wood 0.45 · brick 0.35 · concrete 0.30 · glass 0.08 · metal 0.60 | **NO SOURCE — declared free** | No source found that tabulates diffuse reflectance of *interior* finishes at lidar wavelengths. What IS sourced is **how reflectance maps to range**: the UST-10LX is specified to 10 m against white paper and 4 m at 10% diffuse (Hokuyo spec §2-2, §4), and `effective_range()` interpolates between those anchors. **So the mapping is measured and the input to it is a guess.** Nearest candidate for real numbers: the **SLUM** spectral library (Kotthaus, Smith, Wooster, Grimmond, *ISPRS J. Photogramm. Remote Sens.* **94**, 194–212, 2014), 74 impervious urban material samples at 300–2500 nm — but they are exterior materials and the per-sample list was not confirmed to contain painted plasterboard |

### LoS / NLoS as a modelling choice — two papers Will added, 2026-09-10

| Claim available to us | Source | Status and why |
| --- | --- | --- |
| **NLoS is not just extra dB — it is a STEEPER PATH-LOSS EXPONENT.** 3GPP Case 1: `α_L = 2.09`, `α_NL = 3.75`, with `A_L = 10^-10.38`, `A_NL = 10^-14.54`, breakpoint `d1 = 0.3 km` | Ding, Wang, López-Pérez, Mao, Lin, *Performance Impact of LoS and NLoS Transmissions in Dense Cellular Networks*, arXiv:1503.04251v3 (2015), §VII numerical parameters, citing 3GPP Tables A.1-3, A.1-4, A.1-7 | **FOUND, DELIBERATELY NOT USED.** This is the *outdoor stochastic* form: LoS is a probability that falls with distance, `Pr_L(r)`, because you cannot know where the buildings are. CommsEv now computes LoS **geometrically** from actual walls, which is strictly better information for a maze — and applying a steeper exponent *as well as* the per-wall loss would double-count the same obstruction. Keep this for the day there is outdoor or terrain geometry too coarse to trace: then `Pr_L(r)` is the right tool and these are the right numbers. Caveat either way: cellular, ~2 GHz, hundreds of metres, 0.3 km breakpoint - the STRUCTURE transfers to a 20 m room, the VALUES do not |
| **Coverage first improves and then DEGRADES as a network is densified**, because at short range the interference becomes line-of-sight too | Same paper, abstract and §VII: *"the network coverage probability first increases with the increase of the base station density, and then decreases as the network becomes denser"* | **FOUND, NOT USED — and it is a warning about our own routing axis.** CommsEv's contention model already says more agents on a band is worse. This says something sharper: once geometry exists, adding links is not monotonically good, because **your link may be NLoS while the interferer's path to you is LoS**. A mesh in a maze may be worse than a star. We have never tested that and the model can now express it |
| **Co-channel interference hurts the NLoS link far more than the LoS one.** Measured: an NLoS link sharing a channel with a LoS link scored 68%; moved to a non-overlapping channel it scored **89%** | Fahad & Bulut, *Channel Matters: Exploring LoS/NLoS Channel Effects on WiFi Sensing Performance*, Virginia Commonwealth University, Table II. ESP32 CSI, 2.4 GHz, indoor, walls | **SUGGESTIVE, NOT A NUMBER WE CAN USE.** The metric is *activity-recognition accuracy*, not PDR, so it cannot be lifted into the link model. What it supports is a principle, in the right band and the right kind of room: **the agent already round a corner is the one congestion hurts most**, because it has the least margin left. Our contention model charges every link the same. It also says channel choice matters *more* for the NLoS agent, which is a reason for frequency agility that has nothing to do with jamming |

### Terrain-referenced navigation — sourced, not built

Wanted for the GNSS-denied story and for a future contoured scene. **Nothing
implemented.**

| Claim available to us | Source |
| --- | --- |
| TRN accuracy has a **Cramér–Rao lower bound that is a function of the terrain gradient along the trajectory** — `H_t = ∇h(x_t)` in the matrix Riccati recursion. Flat terrain carries **no position information**: `H_t → 0`, the measurement update contributes nothing, and covariance grows at the process-noise rate | Niclas Bergman, *Recursive Bayesian Estimation: Navigation and Tracking Applications*, Linköping Dissertation No. 579, 1999, **Ch. 7 §7.2–7.3**; the flat-terrain statement at **p. 7**. Simulated **12.2 m CEP** over a 25-minute Swedish-terrain track (**§2.4, p. 18**), which also cites 50 m CEP and 75 m from field tests. **Caveat: airborne radar-altimeter formulation — the `∇h` structure transfers to a ground robot, the numbers do not** |
| TERCOM / SITAN / DSMAC are established GNSS-denied methods, and TERCOM *"struggles in areas with uniform or featureless landscapes"* | Jarraya et al., *GNSS-denied unmanned aerial vehicle navigation*, **Satellite Navigation 6:9, 2025**, §4.1.1. Open access. **Qualitative only — no accuracy figures in metres** |
| *"TRN is not affected by external interference, but its performance can be degraded in flat and repetitive terrain"* | Lee et al., *A Pragmatic Approach to the Design of Advanced Precision Terrain-Aided Navigation for UAVs*, **Remote Sensing 12(9):1396, 2020**, Introduction. Clean and quotable |
| TERCOM as a fielded cruise-missile guidance aid | J. P. Golden, *Terrain Contour Matching (TERCOM): A Cruise Missile Guidance Aid*, **Proc. SPIE 0238**, 1980, DOI 10.1117/12.959127. **Author/title/volume/year/DOI confirmed; page 10 is a start page only — SPIE is bot-walled and the abstract could not be read** |

**Answering the question directly:** yes, the literature bounds TRN accuracy as a
function of terrain relief — as a CRLB in terms of `∇h`, not as a scalar
"X metres for Y metres of relief". I would not trust a source claiming the latter.

**Honest total (live files, Sep 2026): 35 of 57 declared quantities carry a
source. 22 do not.** Every unsourced one above is a declared free parameter,
not a disguised guess — but a result that turns on any of them must say so.

---

## Local obstacle avoidance (added 2026-09-11, IN USE — one citation UNVERIFIED)

Added because nothing consumed the sensors. The model published scans, drew
them, and let a mission steer straight at its station regardless; `blocked()`
stopped the vehicle dead. A fleet with lidar behaved identically to a fleet
without one, which made every sensor in the model decorative.

| Claim | Status | Source |
| --- | --- | --- |
| **Follow-the-Gap**: find the widest opening the vehicle fits through and steer at the bearing in it closest to the goal | **IN USE — CITATION CITED FROM MEMORY, VERIFY** | Sezer & Gokasan, *A novel obstacle avoidance algorithm: Follow the Gap Method*, **Robotics and Autonomous Systems, 2012**. Volume and page numbers **not verified** — check them before quoting any result that depends on this. The algorithm is the standard method on the F1TENTH / RoboRacer platform this project models, which is why it was chosen over VFH or DWA |
| Vector Field Histogram, the older family this belongs to | NOT USED | Borenstein & Koren, IEEE Trans. Robotics and Automation **7(3):278–288**, 1991. **Also from memory — verify.** Kept as the alternative if FTG proves too crude |
| Dynamic Window Approach | NOT USED | Fox, Burgard & Thrun, IEEE Robotics & Automation Magazine **4(1):23–33**, 1997. **From memory — verify.** It reasons about the velocity space, which would need the acceleration model to be measured first |

**Declared free, not from any paper:** the number of bearings sampled across
the field of view (25), the clearance a bearing must show to count as open
(two vehicle radii), and the look-ahead distance. These trade how early a
vehicle commits to a gap against how much it wanders.

**What is NOT claimed.** The method is purely local — one step, no map, no
memory, no plan. It rounds an isolated obstacle and it does **not** solve a
maze: a local method walks into concave traps and sits there. Measured, one
car crossing `scenes/maze.yaml` from (-8, 8) to (8, -8):

| Sensor fit | Result |
| --- | --- |
| none | stuck after **4.4 m**, at the first wall, and never moves again |
| RealSense D435i, 87°, 3 m | **23.7 m** of path, reaches (-0.3, -5.6) |
| Hokuyo UST-10LX, 270°, 10 m | **448.9 m** of path, reaches (-2.3, -5.5) |

The lidar's 449 m for ~15 m of progress is the honest signature of a local
method with no memory: it escapes the first obstacles and then oscillates.
**A real planner is still the missing piece.** What this buys is that sensor
fit is now a difference in BEHAVIOUR a mission outcome can measure, which is
what the sensor-fit experiment always needed.

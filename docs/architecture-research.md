# Beyond the nine — architectures the literature has that we don't

Prompted by Will (1 Sep): are there more than 3 authority models and 3
topologies, including experimental ones? Yes to both. This is the survey.

## What Zhou et al. 2020 gave us (the paper Will supplied)

*Joint Channel Assignment and Power Allocation for Multi-UAV Communication*,
Zhou, Chen, Hong, Jin & Shi, arXiv:2008.08212 (IEEE SPAWC 2020). A GCU controls
K UAVs over a limited set of channels. Four things we took:

1. **Adjacent Channel Interference with a correlation coefficient.** Their
   `mu(f1,f2)` has exactly three properties: 0<=mu<=1, symmetric, mu=1 at zero
   separation, mu->0 as separation grows; "proportional to the intensity of ACI
   and can be measured in practical systems". **Adopted** - our band separation
   was a binary +/-0.5 MHz test, which is far too clean. Now graded, with the
   rolloff parameterised by adjacent-channel rejection (30 dB/channel, a
   commodity receiver figure, declared as a free parameter).
   *Consequence:* a jammer 5 MHz off a 20 MHz channel still delivers ~18% of
   its power. Band separation is now a matter of degree, not a switch.
2. **The near-far problem.** "If d_s,k > d_s,m, the high power signals for UAV
   k would leak into adjacent channels and interfere with the communication
   between GCU and UAV m." A CLOSE agent on an adjacent channel can swamp a
   DISTANT one. Not yet modelled - queued, and now cheap given mu.
3. **Path loss validation.** Their PL = 20lg(f) + 20lg(d) + 32.4 + eta is the
   same Friis-based form we use, which is a useful independent check. They add
   an excess term: **eta_L = 3 dB (LoS), eta_N = 23 dB (NLoS)** - concrete,
   citable numbers for an LoS/NLoS split we do not yet have.
4. **Max-min fairness.** They maximise the MINIMUM SINR across the fleet, not
   the mean, because a swarm is governed by its worst link. **Adopted** - the
   frame now reports `worst_link`, so experiments can score the worst case
   instead of hiding it in an average.

Their simulation shape also validates Will's out-and-back design: as the swarm
flies 1000 m away from the GCU, minimum SINR falls from ~28 dB to ~7 dB, and
more radiation sources push it lower. That is exactly the gradient the
traverse experiment is meant to produce.

Also worth noting as a future countermeasure: their whole point is that
**channel assignment and power allocation are controllable**. A GCU with a
power budget can allocate it smartly (Hungarian algorithm) instead of evenly.
"Smart allocation vs naive" is a ready-made blue-side countermeasure experiment.

## Topologies we are missing

Our three (star / mesh / tiered) are real but incomplete. The FANET literature
splits decentralized into three distinct forms:

- **UAV ad-hoc** - peer-to-peer. (= our mesh.)
- **Multi-group ad-hoc** - several INDEPENDENT clusters, each an ad-hoc network
  internally, with one node per cluster backhauling to the GCS. Different from
  our tiered: the clusters are peer networks, not a strict tree, so a cluster
  survives losing its backhaul entirely.
- **Multi-layer ad-hoc** - agents organised by ALTITUDE, with intra-plane and
  inter-plane links. Directly relevant now that quadcopters are coming: height
  massively extends the radio horizon (a 100 m drone sees 47 km, a ground node
  11.7 km), so an altitude layer is a natural relay tier.

Plus two more from the routing literature:

- **Ring** - each node holds two neighbours; survives any single cut.
- **Delay-tolerant / store-carry-forward (DTN)** - if you cannot transmit, you
  *carry* the data and forward it when a link exists. **The most interesting
  omission for a jamming project**: it converts a comms problem into a mobility
  problem, and it is genuinely unjammable in the usual sense. A "data mule"
  agent that physically ferries orders past a jammed region would be a striking
  result.

FANET routing families for reference: topology-based (proactive/reactive/
hybrid), position/geographic, cluster-based, swarm-intelligence (ACO/PSO),
hierarchical, delay-tolerant, multipath, hybrid.

## Authority models we are missing

- **Mission Command / intent-based ("centralized control, decentralized
  execution").** The doctrinal answer to contested comms, and we do not model
  it. NATO JAPCC's C2-resilience essay recommends exactly this: "emphasize
  commander's intent over detailed orders", "empowering lower-level staff to
  respond effectively without direct orders", and designing for **graceful
  degradation** - systems that "retain some function after critical processes
  are disrupted" rather than collapsing. Critically, an intent-commanded agent
  that loses its link is **not** decapitated: it already knows the goal and
  keeps pursuing it. In our terms it would score high `commanded_fraction`
  under jamming without being decentralized - a genuinely different point in
  the space, and arguably the most important one to model.
- **Adaptive / self-healing hierarchy** - dynamic re-parenting to the best
  available parent (Will's own proposal). Cluster-head election is the
  networking analogue.
- **Consensus-based** - agents converge on shared decisions by agreement;
  Byzantine-tolerant variants matter when an agent may be spoofed.
- **Market / auction-based task allocation** - agents bid for tasks (CBBA and
  descendants). Robust, no central allocator, degrades gracefully as bidders
  drop out. Active research area for UAV swarms including SAR replanning.
- **Stigmergic / emergent** - flocking rules with no explicit command at all.

## Experimental architectures

- **SDN-based FANET** - a centralized *control plane* over a distributed *data
  plane*. This is literally our authority/topology separation implemented in a
  real networking stack, which is strong external validation that the split is
  a real design axis rather than our invention.
- **Blockchain-coordinated swarms** - a distributed ledger for tamper-resistant
  coordination; proposed against injection, spoofing, eavesdropping and
  jamming, and as an information market incentivising cooperation among
  self-interested robots. Costly in bandwidth, which makes it a fascinating
  trade under jamming.
- **Swarm-intelligence routing** (ACO/PSO) - routes discovered by emergent
  optimisation rather than a protocol.
- **Game-theoretic / Stackelberg defence** - attacker and defender as players;
  already in the project library for anti-spoofing.
- **RIS (reconfigurable intelligent surfaces)** - steer/null signals with a
  programmable surface; from the jamming survey.

## Recommendation

Do NOT add all of these. Three are worth the cost, in order:

1. **Mission Command / intent-based authority** - it is the doctrinal answer to
   the exact problem this project studies, and its absence is conspicuous.
2. **Multi-layer (altitude) topology** - arrives naturally with quadcopters and
   exploits the horizon effect already modelled.
3. **DTN / store-carry-forward** - the most distinctive anti-jam idea available
   and a genuinely different resilience mechanism.

The rest belong in the write-up as related work, not in the model.

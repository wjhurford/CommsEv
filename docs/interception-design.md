# Design — command interception over the air ("hearing" the other side)

**Status: designed, not built. The build is scheduled with the cell-terminals
step.** The requirement: a red (or blue) terminal should be able to **hear
commands sent over a frequency**, as a real receiver on that band does, and
then act on what it heard. This is COMINT / electronic support, and it is the
same physical act a **reactive jammer** performs — Priyadarshani et al. (IEEE
Access 6, 2025) define a reactive jammer as one that "monitors channels and
activates only upon detecting legitimate transmissions." Interception is
therefore not a bolt-on; it is the sensing half the reactive-jammer ladder
already needs.

It is built with the cell-terminals step because it needs the blue/red/white
cells to have somewhere to deliver what was heard. It is recorded here so that
the seams go in now and the later build is small.

## The physical rule (one rule, reusing the RF model)

A transmission is heard by a receiver when, on the transmitter's band, the
receiver's link to the transmitter is usable — exactly the `rf_link()` verdict
already computed every frame. "Who can hear this command" is therefore not new
physics: it is the set of agents for whom `rf_link(sender, listener)` on that
band is not `down`. An eavesdropper simply need not be the intended recipient.

Three factors gate whether a heard command is *usable* to the listener:

1. **Range / SINR** — already modelled. Out of range means not heard. This is
   why a low-power, directional, or fibre-tethered link is hard to intercept:
   the README's EMCON and fibre ideas become effective here.
2. **Band** — already modelled. A listener on a different band hears nothing
   (the same band-separation rule as jamming).
3. **Encryption** — the new element. A command carries an `encrypted` flag
   (per network, from the scene/fleet `encrypted:` field that already exists on
   blue). An eavesdropper hears an encrypted transmission as *traffic*
   (presence, direction, timing — traffic analysis is real intelligence) but
   not its *content*. Plaintext is fully readable. This makes encryption a
   modelled defence with a real cost/benefit, per the README.

## Data seams to add (small, now)

- Every command already flows through the retask spool as one line. Wrap it
  with sender + network + band + encrypted when it is issued, so the simulation
  knows what went over the air and on what frequency. (Today a line is
  anonymous; it gains a provenance envelope.)
- The frame gains an `intercepts` list: for each command that crossed the air
  this tick, which agents were physically able to hear it, and whether they
  obtained content (plaintext) or only metadata (encrypted). This is the exact
  analogue of `attacks_active` for the ES/COMINT side.

## What the cells do with it (built with the terminals)

- **Red cell** subscribes to intercepts on blue's band: heard blue commands
  print in the red terminal ("t=12.3  overheard blue: REOBJECTIVE car1 pursue
  car3"), plaintext or "[encrypted — traffic only]". The red cell can then act;
  most naturally, it retasks a mobile jammer toward the sender
  (`REOBJECTIVE jam1 pursue car1`), i.e. a **reactive jammer** driven by
  interception.
- **Blue cell** hears red's band symmetrically (if red transmits in the
  clear), so detection of the jammer's own emissions is available to blue.
- **White cell** (umpire) hears everything: ground truth for grading.

## Grading hook (later, ties to the ML step)

Interceptability is a measurable cost of transmitting: a run can be scored on
how much of blue's command traffic red could read, which is exactly what
EMCON, low-probability-of-intercept links and encryption are meant to reduce.
This is a natural axis for the gamified levels.

## Why not now, in full

The listening physics and the envelope are cheap and go in with the seams
above. "Act on what was heard", however, needs the red cell to exist as a
distinct, scoped terminal with its own view, which is the next step but one.
Building interception's payoff before the cells exist would mean inventing a
throwaway home for the heard commands. Sequence: Network tab → cell terminals
(+ interception delivery) → reactive jammer driven by it.

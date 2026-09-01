# Is the jamming model actually useful? An honest, sourced answer

Will asked the fair question. Short answer: **yes as a network- and
command-resilience testbed, no as a physical-layer RF simulator** — and the
distinction matters for what we may claim from it. Every decision below is
tied to the project's own library.

## What the model does, and why each choice is defensible

**A jammer is same-band interference power summed into the SINR denominator.**
This is the textbook definition, not a shortcut. Priyadarshani et al.
(*Jamming Intrusions in Extreme Bandwidth Communication*, IEEE Access 6, 2025)
define jamming as "transmitting signal on the same frequency band used by the
legitimate system," and the governing quantity throughout the GNSS literature
is the **jamming-to-signal ratio (JSR/J-S)** — e.g. detectors validated "for
JSR above −10 dB" and jammers detected "at JSR of 45 dB" (Sokolova et al.,
*sensors* 2024, 24 04210). Our `rf_link()` computes exactly a
signal-versus-(noise+interference) ratio, so JSR falls straight out of it.
**Verdict: the core quantity is the right one.**

**A jammer is an ordinary agent (a point source, omnidirectional, on a band),
not a global flag.** This mirrors Tedeschi & Di Pietro (*SpaCCS* 2021), whose
model is a jammer "placed statically… emits a high-power signal on a particular
bandwidth… omnidirectional antenna, so the emitted interference propagates
360° around its location." Their received-power law is a path-loss model of
distance — `Pr(d) = Θ(d) + Ψ(d) + φ(t)` — whose deterministic term Θ(d) is
exactly our log-distance path loss. **Verdict: modelling the jammer as an
ordinary emitter obeying the same path loss as any signal is the accepted
approach, and it is what lets "a jammer degrades comms" and "comms loss
strips command authority" be one causal chain rather than two bolted-on
effects.** That chain is the thing this project is actually about, and few
off-the-shelf sims represent it.

**The scene owns the noise floor; the jammer's rise is measured from it.**
Tedeschi names a boundary RSS "P_ω expected at the boundary of the jammed
area" against ambient — a jammed region is defined relative to the resting
floor. Our `baseline_dbm` vs experienced `noise_floor_dbm` is that same
relative reading. **Verdict: correct framing.**

**Band separation is a hard defence.** Off-band jammers contribute nothing in
our model. The survey lists frequency-hopping as a primary anti-jam, and names
the *follow-on jammer* as its specific counter — both only meaningful if band
separation matters. **Verdict: correct, and the seam for FHSS work later.**

**The observables we emit are the ones detectors actually consume.** The
survey's detection methods are RSS, Packet Error Rate / PDR, and noise-level
measurement; the ML detectors it cites (random forest at ~97.5% accuracy) take
"RSS, bad packet ratio, packet delivery ratio, clear channel assessment." We
already produce per-agent RSS, PDR and noise floor every frame. **Verdict:
this is the strongest single argument that the tool is useful — its telemetry
IS the feature set the Kalman/ML step will consume. It was built to be a
substrate for detection, and it is one.**

## Where the model is genuinely weak — do not oversell it

- **The PDR curve is a logistic stand-in, not a modulation/coding curve.**
  Real link quality is a BER-vs-SINR curve set by modulation and FEC; the
  survey names LDPC and Reed-Solomon as anti-jam coding. We have none of that.
  A result that turns on the *exact* PDR at a given SINR is not trustworthy;
  a result about *trends and ordering* (this topology survives, that one does
  not) is.
- **No processing gain / spread spectrum.** DSSS/FHSS give a real dB margin
  against a jammer; we don't model it, so we currently *understate* a
  spread-spectrum radio's resilience. This is the biggest single gap before
  any "anti-jam radio" claim.
- **Omnidirectional only.** No beamforming, no directional antennas, no
  spatial nulling or MIMO interference rejection — all of which the survey and
  the mmWave papers treat as central. Our range is therefore a *circle*; a
  real directional jammer or victim is not. (This is why the README's
  "directionality" idea is a real future axis, not polish.)
- **Constant jammer only.** We model the *proactive/constant* class. Reactive,
  random, deceptive, follow-on and smart jammers (the survey's taxonomy, and
  the README's ladder) are not built — they are behaviours on the jammer
  agent, and the honest levels ladder is exactly that taxonomy.
- **Point isotropic, single number.** The Baltic Sea field trial (sensors
  2024) found a jammer's "area of influence exceeds a radius of three
  kilometres, although its effect is not uniform." Our range ring is a nominal
  contour and must be labelled as such — real jammed areas are ragged.
- **Defaults are unsourced.** Noise floor and path-loss exponent are
  defensible defaults, not measurements — the provenance rule already flags
  them, and any load-bearing result must say so.

## The honest verdict

As a **testbed for network structure and command resilience under contested
comms** — which is what DEADBAND is for — the model is sound and grounded: the
right quantity (JSR), the right causal chain (jamming → link loss → authority
loss), and the right observables for the detection/ML work to come. As a
**physical-layer RF fidelity model** it is deliberately coarse, and we should
never present a specific dB or PDR figure as if it were. Build the levels
ladder and processing-gain next if we want to strengthen the anti-jam side;
keep stating the free parameters.

### Sources (project library)
- Priyadarshani et al., *Jamming Intrusions in Extreme Bandwidth
  Communication: A Comprehensive Overview*, IEEE Access 6, 2025. (jammer
  taxonomy; detection metrics; anti-jam techniques)
- Tedeschi & Di Pietro, *SpaCCS* 2021. (constant AWGN jammer, omnidirectional,
  path-loss RSS model, jammed-area boundary)
- Sokolova et al., *sensors* 24(04210), 2024. (JSR, GNSS jamming detection,
  Baltic Sea field trial, "effect is not uniform")

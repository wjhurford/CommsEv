# Usefulness of the jamming model — a sourced assessment

The question was raised in review whether the jamming model is useful. Short
answer: **yes as a network- and command-resilience testbed; no as a
physical-layer RF simulator.** The distinction governs what may be claimed from
it. Every decision below is tied to the project's own library.

## What the model does, and why each choice is defensible

**A jammer is same-band interference power summed into the SINR denominator.**
This is the textbook definition, not a shortcut. Priyadarshani et al.
(*Jamming Intrusions in Extreme Bandwidth Communication*, IEEE Access 6, 2025)
define jamming as "transmitting signal on the same frequency band used by the
legitimate system," and the governing quantity throughout the GNSS literature
is the **jamming-to-signal ratio (JSR/J-S)** — e.g. detectors validated "for
JSR above −10 dB" and jammers detected "at JSR of 45 dB" (Sokolova et al.,
*sensors* 2024, 24 04210). The `rf_link()` function computes exactly a
signal-versus-(noise+interference) ratio, so JSR falls straight out of it.
**Verdict: the core quantity is the right one.**

**A jammer is an ordinary agent (a point source, omnidirectional, on a band),
not a global flag.** This mirrors Tedeschi & Di Pietro (*SpaCCS* 2021), whose
model is a jammer "placed statically… emits a high-power signal on a particular
bandwidth… omnidirectional antenna, so the emitted interference propagates
360° around its location." Their received-power law is a path-loss model of
distance — `Pr(d) = Θ(d) + Ψ(d) + φ(t)` — whose deterministic term Θ(d) is
exactly the log-distance path loss used here. **Verdict: modelling the jammer
as an ordinary emitter obeying the same path loss as any signal is the accepted
approach, and it is what lets "a jammer degrades comms" and "comms loss strips
command authority" be one causal chain rather than two bolted-on effects.**
That chain is the subject of this project, and few off-the-shelf simulators
represent it.

**The scene owns the noise floor; the jammer's rise is measured from it.**
Tedeschi names a boundary RSS "P_ω expected at the boundary of the jammed
area" against ambient: a jammed region is defined relative to the resting
floor. The `baseline_dbm` versus experienced `noise_floor_dbm` reading is that
same relative measure. **Verdict: correct framing.**

**Band separation is a hard defence.** Off-band jammers contribute nothing in
the model. The survey lists frequency-hopping as a primary anti-jam measure,
and names the *follow-on jammer* as its specific counter; both are meaningful
only if band separation matters. **Verdict: correct, and the seam for FHSS
work later.**

**The observables emitted are the ones detectors consume.** The survey's
detection methods are RSS, Packet Error Rate / packet-delivery ratio (PDR), and
noise-level measurement; the ML detectors it cites (random forest at ~97.5%
accuracy) take "RSS, bad packet ratio, packet delivery ratio, clear channel
assessment." The model already produces per-agent RSS, PDR and noise floor
every frame. **Verdict: this is the strongest single argument for the tool's
usefulness. Its telemetry is the feature set the Kalman/ML step will consume.
It was built as a substrate for detection, and it functions as one.**

## Where the model is weak — not to be oversold

- **The PDR curve is a logistic stand-in, not a modulation/coding curve.**
  Real link quality is a BER-vs-SINR curve set by modulation and FEC; the
  survey names LDPC and Reed-Solomon as anti-jam coding. The model has none of
  that. A result that turns on the *exact* PDR at a given SINR is not
  trustworthy; a result about *trends and ordering* (this routing survives,
  that one does not) is.
- **No processing gain / spread spectrum.** DSSS/FHSS give a real dB margin
  against a jammer; the model does not include it, so it currently
  *understates* a spread-spectrum radio's resilience. This is the largest
  single gap before any "anti-jam radio" claim.
- **Omnidirectional only.** No beamforming, no directional antennas, no
  spatial nulling or MIMO interference rejection — all of which the survey and
  the mmWave papers treat as central. The modelled range is therefore a
  *circle*; a real directional jammer or victim is not. (This is why the
  README's "directionality" idea is a real future axis, not polish.)
- **Constant jammer only.** The model represents the *proactive/constant*
  class. Reactive, random, deceptive, follow-on and smart jammers (the survey's
  taxonomy, and the README's ladder) are not built; they are behaviours on the
  jammer agent, and the levels ladder is exactly that taxonomy.
- **Point isotropic, single number.** The Baltic Sea field trial (sensors
  2024) found a jammer's "area of influence exceeds a radius of three
  kilometres, although its effect is not uniform." The modelled range ring is
  a nominal contour and must be labelled as such; real jammed areas are
  ragged.
- **Defaults are unsourced.** Noise floor and path-loss exponent are
  defensible defaults, not measurements. The provenance rule already flags
  them, and any load-bearing result must say so.

## Verdict

As a **testbed for network structure and command resilience under contested
comms**, which is what CommsEv is for, the model is sound and grounded: the
right quantity (JSR), the right causal chain (jamming → link loss → authority
loss), and the right observables for the detection/ML work to come. As a
**physical-layer RF fidelity model** it is deliberately coarse, and a specific
dB or PDR figure must never be presented as if it were one. To strengthen the
anti-jam side, build the levels ladder and processing gain next, and keep
stating the free parameters.

### Sources (project library)

- Priyadarshani et al., *Jamming Intrusions in Extreme Bandwidth
  Communication: A Comprehensive Overview*, IEEE Access 6, 2025. (jammer
  taxonomy; detection metrics; anti-jam techniques)
- Tedeschi & Di Pietro, *SpaCCS* 2021. (constant AWGN jammer, omnidirectional,
  path-loss RSS model, jammed-area boundary)
- Sokolova et al., *sensors* 24(04210), 2024. (JSR, GNSS jamming detection,
  Baltic Sea field trial, "effect is not uniform")

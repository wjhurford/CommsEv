# The band strip

A row of frequency buttons above the timeline. Its job: make the spectrum
legible — which band each link is on, and which band is being jammed right now.

- **All** — every link, every band (the default).
- A **comms band** (e.g. 2.4 GHz) — only links on that band. If two networks
  used different bands, this is how you'd see them apart.
- **GNSS 1575** — hides the comms links (GNSS carries no links between agents;
  it is satellite-to-receiver). On this band the story is the drift, so the
  belief ghosts are what you watch.

A button turns **orange and bold** while a jammer is emitting on that band,
updated every frame — so "what are we jamming, and when" is answered by
glancing at the strip rather than reading the Contested tab. A GNSS-band jammer
lights the GNSS button; a comms-band jammer lights the comms button; retune the
jammer with `JAM jam1 band <MHz>` and the highlight follows.

Built to grow: when per-agent multi-band radios and more bands land (see
docs/spectrum-and-sensors-reference.md), each new band simply appears as
another button.

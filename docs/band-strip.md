# The band strip

A row of frequency buttons above the timeline. Its purpose is to make the
spectrum legible — which band each link is on, and which band is being jammed
at the current moment.

- **All** — every link, every band (the default).
- A **comms band** (e.g. 2.4 GHz) — only links on that band. If two networks
  used different bands, this view separates them.
- **GNSS 1575** — hides the comms links (GNSS carries no links between agents;
  it is satellite-to-receiver). On this band the relevant quantity is the
  drift, so the belief ghosts are the thing to observe.

A button turns **orange and bold** while a jammer is emitting on that band,
updated every frame — so the question of what is being jammed, and when, is
answered by glancing at the strip rather than reading the Contested tab. A
GNSS-band jammer lights the GNSS button; a comms-band jammer lights the comms
button; retuning the jammer with `JAM jam1 band <MHz>` moves the highlight
accordingly.

Built to grow: when per-agent multi-band radios and more bands land (see
`docs/spectrum-and-sensors-reference.md`), each new band appears as another
button.

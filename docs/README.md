# docs/ — where to start

Read in this order if you are new. Everything else is reference or history.

## Start here

| Doc | Why |
| --- | --- |
| `HANDOVER-DEMO.md` | The ten-minute demo, step by step, with what you should see. Written for the incoming PhD student against the working tree. |
| `commsev-handover.md` | The one-page brief: what the testbed is, what it has shown, where it sits against Gazebo / ns-3 / CORNET, and the open question of tool vs result vs method. |
| `vocabulary.md` | The three layers — scene, fleet, mission — and why "scenario" was retired. Authoritative. |
| `REVIEW-2026-09-01.md` | Adversarial review: what is weak, what is broken, what is overstated. Read the top three items before touching the model. |
| `ROADMAP.md` | The agreed build order, what is done and what is next. Supersedes `BACKLOG.md` where they disagree. |

## The model — design and grounding

| Doc | Covers |
| --- | --- |
| `jamming-model-justification.md` | Is the jamming model useful? A sourced answer, and the scope line it implies. |
| `jamming-effects-research.md` | What actually happens to an agent that is jammed: comms failsafes vs GNSS drift, with the literature. |
| `gnss-drift-model.md` | The belief-vs-truth drift model as built. |
| `contested-background.md` | What a scene declares about its radio environment, and why. |
| `interception-design.md` | Command interception over the air — red "hearing" blue. |
| `cells-and-network.md` | The white / blue / red cells and the Network tab. |
| `band-strip.md` | The band strip in the Console. |
| `experiments-design.md` | The sweep harness: what an experiment file declares and what it produces. |
| `architecture-research.md` | Command architectures the literature has that the model does not yet. |
| `spectrum-and-sensors-reference.md` | Real frequencies, jamming targets, and the sensors on the vehicles. |
| `ros2-and-missions.md` | Getting the same model up as ROS 2 nodes, and writing mission behaviour scripts. |

`../SOURCES.md` is the ledger of every physical constant and its status;
`../COMMANDS.md` is the terminal grammar.

## History

`PATCH-05` to `PATCH-09-CHECKS.md` record what each patch changed and how it
was click-checked. `BACKLOG.md` is the running list of decisions and deferred
work. Three docs are banner-marked **SUPERSEDED** — `CHECKLIST.md`,
`maps-missions-and-retasking.md`, `writing-a-mission.md` — and are kept
because the decisions in them explain the shape of what replaced them. Do not
implement against them.

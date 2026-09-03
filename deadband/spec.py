"""
Loading and checking a Deadband scenario file.

This is the spine of the whole framework. A scenario file is the single source
of truth for an experiment; everything else reads it. So this module has two
jobs and no others:

  1. Load the YAML into plain Python data.
  2. Tell you everything wrong with it, in one go, in language a human can act
     on — including which physical parameters have no source.

It deliberately knows nothing about Gazebo, ROS or radios. Keeping it that way
is what lets the GUI, the docs and the tests all use it without dragging in a
robotics stack.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass, field
from pathlib import Path

import yaml

# A "quantity" is our provenance-carrying number: {value, unit, source}.
# Anywhere in the file that looks like this gets checked.
QUANTITY_KEYS = {"value", "unit", "source"}

# Sections a scenario must have. Deliberately short — the schema will grow, but
# every addition here is a thing every future user is forced to write.
REQUIRED_TOP_LEVEL = ["spec_version", "name", "arena", "agents"]

VALID_BOUNDARIES = {"solid", "open", "absorbing"}
# AUTHORITY = who decides. ROUTING = how packets travel. They are independent
# axes (docs/vocabulary.md, the authority x topology matrix), so they are
# validated separately. `topology` is the legacy name for authority and is
# still accepted so older files load.
VALID_AUTHORITIES = {"centralized", "decentralized", "hierarchical"}
VALID_ROUTINGS = {"star", "mesh", "tiered"}
VALID_LEADER_LOSS = {"fallback", "strand"}
# What a vehicle does with no reachable commander. `intent` is NATO mission
# command - decentralized execution on delegated intent (AJP-3 3.8, 3.11);
# `continue` is its legacy alias. See stub_telemetry.LINK_LOSS_KEEPS_GOING.
VALID_LINK_LOSS = {"hold", "intent", "continue"}
VALID_TOPOLOGIES = VALID_AUTHORITIES        # legacy alias


@dataclass
class Quantity:
    """One physical parameter and where its value came from."""
    path: str          # e.g. "agents[rover1].dimensions.length"
    value: object
    unit: str
    source: str

    @property
    def sourced(self) -> bool:
        return bool(self.source and self.source.strip())


@dataclass
class Report:
    """Everything we learned about a scenario file."""
    path: Path
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    quantities: list[Quantity] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors

    @property
    def unsourced(self) -> list[Quantity]:
        return [q for q in self.quantities if not q.sourced]

    @property
    def missing_values(self) -> list[Quantity]:
        return [q for q in self.quantities if q.value is None]


def _walk(node, path, report: Report) -> None:
    """
    Recursively find every provenance-carrying quantity in the file.

    We identify one structurally rather than by name: any dict that has 'value'
    and 'source' keys is a quantity. That means new parameters are picked up
    automatically without this function ever being edited.
    """
    if isinstance(node, dict):
        if "value" in node and "source" in node:
            report.quantities.append(
                Quantity(
                    path=path,
                    value=node.get("value"),
                    unit=node.get("unit", ""),
                    source=node.get("source", ""),
                )
            )
            # A quantity is a leaf — don't descend into it.
            unexpected = set(node) - QUANTITY_KEYS
            if unexpected:
                report.warnings.append(
                    f"{path}: unexpected keys in a quantity: {sorted(unexpected)}"
                )
            return
        for key, value in node.items():
            _walk(value, f"{path}.{key}" if path else str(key), report)

    elif isinstance(node, list):
        for i, item in enumerate(node):
            # Use the item's id for the path where it has one — far more
            # readable than agents[0] when you're reading an error message.
            label = item.get("id") if isinstance(item, dict) and "id" in item else i
            _walk(item, f"{path}[{label}]", report)


def _required_for(doc: dict) -> list[str]:
    """Which top-level sections THIS file must carry, by its layer.

    The three-layer model (docs/vocabulary.md): a SCENE owns the world, a
    FLEET owns the agents, a MISSION names both and owns the command - so a
    mission is not missing an arena, its scene has it. A file with no `kind`
    and no base reference is a legacy self-contained scenario and still owes
    everything.
    """
    kind = doc.get("kind")
    has_base = bool(doc.get("scene") or doc.get("map") or doc.get("fleet"))
    required = ["spec_version", "name"]
    if kind == "scene":
        required.append("arena")
    elif kind == "fleet":
        required.append("agents")
    elif kind == "mission" or has_base:
        pass                    # world and fleet arrive through the chain
    else:
        required += ["arena", "agents"]
    return required


def _check_structure(doc: dict, report: Report) -> None:
    """Structural rules. Add to this as the schema settles."""
    for key in _required_for(doc):
        if key not in doc:
            report.errors.append(f"missing required top-level section: '{key}'")

    arena = doc.get("arena") or {}
    for face, kind in (arena.get("boundaries") or {}).items():
        if kind not in VALID_BOUNDARIES:
            report.errors.append(
                f"arena.boundaries.{face}: '{kind}' is not one of {sorted(VALID_BOUNDARIES)}"
            )

    networks = doc.get("networks") or {}
    for net_name, net in networks.items():
        net = net or {}
        # authority (or its legacy alias topology/architecture) must be named
        # and valid - a silent typo here changes who commands whom.
        authority = (net.get("authority") or net.get("architecture")
                     or net.get("topology"))
        if authority not in VALID_AUTHORITIES:
            report.errors.append(
                f"networks.{net_name}.authority: '{authority}' is not one of "
                f"{sorted(VALID_AUTHORITIES)}"
            )
        routing = net.get("routing")
        if routing is not None and routing not in VALID_ROUTINGS:
            report.errors.append(
                f"networks.{net_name}.routing: '{routing}' is not one of "
                f"{sorted(VALID_ROUTINGS)}"
            )
        leader_loss = net.get("leader_loss")
        if leader_loss is not None and leader_loss not in VALID_LEADER_LOSS:
            report.errors.append(
                f"networks.{net_name}.leader_loss: '{leader_loss}' is not one "
                f"of {sorted(VALID_LEADER_LOSS)}"
            )
        # A hierarchy needs squads to be a hierarchy at all.
        if authority == "hierarchical" and not net.get("squads"):
            report.warnings.append(
                f"networks.{net_name}: authority is 'hierarchical' but no "
                f"squads are declared - it will behave as a star on the "
                f"coordinator"
            )
        for sq_name, spec_ in (net.get("squads") or {}).items():
            spec_ = spec_ or {}
            if not spec_.get("leader"):
                report.errors.append(
                    f"networks.{net_name}.squads.{sq_name}: no leader")

    # Cross-references: every agent must point at things that exist.
    agent_ids = set()
    radio_defs = set(doc.get("radios") or {})

    for agent in doc.get("agents") or []:
        aid = agent.get("id")
        if not aid:
            report.errors.append("an agent has no 'id'")
            continue
        if aid in agent_ids:
            report.errors.append(f"duplicate agent id: '{aid}'")
        agent_ids.add(aid)

        net = agent.get("network")
        if networks and net not in networks:
            report.errors.append(
                f"agent '{aid}': network '{net}' is not defined in the networks section"
            )

        for radio in agent.get("radios") or []:
            if radio not in radio_defs:
                report.errors.append(
                    f"agent '{aid}': radio '{radio}' is not defined in the radios section"
                )

    # A coordinator must be a real agent, or the launcher has nothing to talk to.
    for net_name, net in networks.items():
        coordinator = (net or {}).get("coordinator")
        if coordinator and coordinator not in agent_ids:
            report.errors.append(
                f"networks.{net_name}.coordinator: '{coordinator}' is not an agent id"
            )


def load(path: str | Path) -> Report:
    """Load and check a scenario file. Never raises on a bad file — the
    problems come back in the report so we can show all of them at once."""
    path = Path(path)
    report = Report(path=path)

    try:
        raw = path.read_text(encoding="utf-8")
    except OSError as exc:
        report.errors.append(f"cannot read file: {exc}")
        return report

    try:
        doc = yaml.safe_load(raw)
    except yaml.YAMLError as exc:
        report.errors.append(f"not valid YAML: {exc}")
        return report

    if not isinstance(doc, dict):
        report.errors.append("the top level of a scenario must be a mapping")
        return report

    # RESOLVE THE LAYERS FIRST. A composed run (scene + fleets + overrides) is
    # a PARTIAL document: its `agents:` list carries only what the Setup tab is
    # overriding - a pose, a doctrine - and gets the network, sensors and body
    # from the fleet layer underneath. Validating the partial on its own
    # reported every agent as having no network, which was alarming, wrong, and
    # had nothing to do with the run that then executed perfectly.
    #
    # Anything with a scene/fleet reference is resolved before checking. A
    # single self-contained file resolves to itself, so nothing else changes.
    checked = doc
    if any(k in doc for k in ("scene", "fleet", "fleets", "map")):
        try:
            from tools import stub_telemetry as _st  # type: ignore
        except ImportError:
            import sys as _sys
            _sys.path.insert(0, str(Path(__file__).resolve().parent.parent
                                    / "tools"))
            try:
                import stub_telemetry as _st  # type: ignore
            except ImportError:
                _st = None
        if _st is not None:
            try:
                checked = _st.resolve_doc(dict(doc))
            except Exception as exc:
                report.errors.append(
                    f"cannot resolve this run's scene/fleet layers: {exc}")
                return report

    report.doc = doc  # type: ignore[attr-defined]
    _check_structure(checked, report)
    _walk(checked, "", report)
    return report


def format_report(report: Report) -> str:
    """Human-readable summary. This is what the CLI prints."""
    lines: list[str] = [f"{report.path}", ""]

    if report.errors:
        lines.append(f"ERRORS ({len(report.errors)})")
        lines += [f"  - {e}" for e in report.errors]
        lines.append("")

    if report.warnings:
        lines.append(f"WARNINGS ({len(report.warnings)})")
        lines += [f"  - {w}" for w in report.warnings]
        lines.append("")

    total = len(report.quantities)
    unsourced = report.unsourced
    missing = report.missing_values

    lines.append(f"PROVENANCE  {total - len(unsourced)}/{total} parameters sourced")
    if unsourced:
        lines.append("")
        lines.append(f"  no source ({len(unsourced)}):")
        lines += [f"    {q.path}" for q in unsourced]
    if missing:
        lines.append("")
        lines.append(f"  no value yet ({len(missing)}):")
        lines += [f"    {q.path}" for q in missing]

    lines.append("")
    lines.append("OK" if report.ok else "FAILED")
    return "\n".join(lines)

"""
Command line entry point.

    python3 -m deadband validate default_run.yaml

Exit code 0 if the file is structurally valid, 1 if not. Unsourced parameters
are reported but do NOT fail the check — an honest empty source is a legitimate
state to commit. Missing sources are a number to drive down, not a gate.
"""

import sys

from . import spec


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv

    if len(argv) != 2 or argv[0] != "validate":
        print("usage: python3 -m deadband validate <scenario.yaml>", file=sys.stderr)
        return 2

    report = spec.load(argv[1])
    print(spec.format_report(report))
    return 0 if report.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

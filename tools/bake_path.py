#!/usr/bin/env python3
"""Bake a waypoint YAML into a played-back trajectory file (run ON SHORE).

Usage: python tools/bake_path.py src/planning/planning/prequal.yaml
Writes <name>.baked.yaml next to the source (picked up by the planning
package's data_files glob, so it installs to the share dir).
"""

import sys

sys.path.insert(0, "src/planning")

from planning.utils.bake import bake, save_bake  # noqa: E402


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(2)
    src = sys.argv[1]
    out = (
        src[: -len(".yaml")] + ".baked.yaml"
        if src.endswith(".yaml")
        else src + ".baked.yaml"
    )
    doc = bake(src)
    save_bake(doc, out)
    legs = sum(1 for i in doc["items"] if i["type"] == "leg")
    pursuits = len(doc["items"]) - legs
    total = sum(
        i["duration"] + i["pause_after"] for i in doc["items"] if i["type"] == "leg"
    )
    print(
        f"Baked {out}: {legs} leg(s), {pursuits} pursuit(s), "
        f"{total:.1f}s of splined trajectory"
    )


if __name__ == "__main__":
    main()

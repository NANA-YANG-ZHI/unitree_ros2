"""
Run plot_excitation_vis.py's plot_excitation_vis() over every
*_sportmodestate.npz in example/data/npz_data/2026_07_21/ (or --npz-dir),
saving one <stem>_excitation_vis.png per bag+segment into this directory
(or --out-dir).

Only needs numpy/matplotlib -- run in the plot-tools-run container.

Usage:
    python plot_all_excitation_vis.py
    python plot_all_excitation_vis.py --npz-dir DIR --out-dir DIR
"""

import argparse
from pathlib import Path

from plot_excitation_vis import DEFAULT_NPZ_DIR, plot_excitation_vis

DEFAULT_OUT_DIR = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--npz-dir", default=str(DEFAULT_NPZ_DIR), help="Directory of *_sportmodestate.npz / *_desired.npz files")
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT_DIR), help="Directory to save PNGs into")
    args = parser.parse_args()

    npz_dir = Path(args.npz_dir)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    stems = sorted(p.name[: -len("_sportmodestate.npz")] for p in npz_dir.glob("*_sportmodestate.npz"))
    if not stems:
        print(f"No *_sportmodestate.npz files found in {npz_dir}")
        return

    fail_count = 0
    for stem in stems:
        out_path = out_dir / f"{stem}_excitation_vis.png"
        print(f"== {stem} ==")
        try:
            plot_excitation_vis(stem, npz_dir, out_path, show=False)
        except Exception as e:
            fail_count += 1
            print(f"== {stem}: FAILED ({e}) ==")

    ok_count = len(stems) - fail_count
    print(f"\nDone: {ok_count} ok, {fail_count} failed, {len(stems)} total")


if __name__ == "__main__":
    main()

"""
Run plot_estimator_vs_sensor_zoom.py's plot_zoom() over every *_contact_estimate.npz
in plot/contact_estimation/contact_estimation_result_npz/, saving one PNG per bag
into this directory.

Only needs numpy/matplotlib -- run in the plot-tools-run container.

Usage:
    python plot_all_estimator_vs_sensor_zoom.py [--t0 25] [--t1 33]
    python plot_all_estimator_vs_sensor_zoom.py --npz-dir DIR --out-dir DIR
"""

import argparse
from pathlib import Path

from plot_estimator_vs_sensor_zoom import plot_zoom

DEFAULT_NPZ_DIR = Path(__file__).resolve().parents[1] / "contact_estimation_result_npz"
DEFAULT_OUT_DIR = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--npz-dir", default=str(DEFAULT_NPZ_DIR), help="Directory of *_contact_estimate.npz files")
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT_DIR), help="Directory to save PNGs into")
    parser.add_argument("--t0", type=float, default=0.0, help="Zoom window start (s)")
    parser.add_argument("--t1", type=float, default=30.0, help="Zoom window end (s)")
    args = parser.parse_args()

    npz_dir = Path(args.npz_dir)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    npz_paths = sorted(npz_dir.glob("*_contact_estimate.npz"))
    if not npz_paths:
        print(f"No *_contact_estimate.npz files found in {npz_dir}")
        return

    fail_count = 0
    for npz_path in npz_paths:
        out_path = out_dir / f"{npz_path.stem}_estimator_vs_sensor_zoom.png"
        print(f"== {npz_path.name} ==")
        try:
            plot_zoom(npz_path, args.t0, args.t1, out_path, show=False)
        except Exception as e:
            fail_count += 1
            print(f"== {npz_path.name}: FAILED ({e}) ==")

    ok_count = len(npz_paths) - fail_count
    print(f"\nDone: {ok_count} ok, {fail_count} failed, {len(npz_paths)} total")


if __name__ == "__main__":
    main()

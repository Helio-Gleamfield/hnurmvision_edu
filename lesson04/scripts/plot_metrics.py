"""Plot the committed, complete runs; no simulated points or discarded outliers."""
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parents[1] / "artifacts"
fig, axes = plt.subplots(2, 1, figsize=(10, 6.2), layout="constrained")
for ax, case, title in zip(axes, ["static", "translation"],
                           ["Static target", "Translating target"]):
    with (root / f"{case}-poses.csv").open() as f:
        poses = list(csv.DictReader(f))
    with (root / f"{case}-forecasts.csv").open() as f:
        predictions = list(csv.DictReader(f))
    origin = int(poses[0]["stamp_ns"])
    ax.plot([(int(r["stamp_ns"]) - origin) / 1e9 for r in poses],
            [float(r["position_error_m"]) * 1000 for r in poses],
            label="Same-frame PnP position error", color="#16804a", lw=1.4)
    ax.plot([(int(r["issued_ns"]) - origin) / 1e9 for r in predictions],
            [float(r["error_m"]) * 1000 for r in predictions],
            label="250 ms delayed prediction error", color="#be279b", lw=1.3)
    ax.set(title=title, xlabel="Time of observation / forecast issue (s)",
           ylabel="3D error (mm)", ylim=(0, 150), xlim=(0, 20))
    ax.grid(alpha=.2)
    ax.legend(loc="upper right", framealpha=.95)
fig.suptitle("Actual simulator runs, fixed observer, known 15 deg armor tilt")
fig.savefig(root / "errors.png", dpi=150)
print(root / "errors.png")

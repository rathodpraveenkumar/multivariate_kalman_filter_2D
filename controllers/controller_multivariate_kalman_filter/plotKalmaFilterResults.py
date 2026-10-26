import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

dataFile = Path(__file__).resolve().parent / "MultivariateKF_results.npz"
data = np.load(dataFile)

t = data["time"]

def plotPosition(axis, gt, gps, pred, est, predStd, estStd, label):
    fig, ax = plt.subplots(figsize=(12, 6))

    ax.plot(t, gt, "k--", label="Ground truth")
    ax.scatter(t, gps, facecolors="none", edgecolors="black",
               label="GPS measurements")
    ax.plot(t, pred, ":", color="orange", label="Prediction")
    ax.plot(t, est, color="tab:blue", label="Posterior estimate")

    ax.fill_between(t, est-estStd, est+estStd,
                    color="tab:blue", alpha=0.2, label="Estimate ±1σ")
    ax.fill_between(t, pred-predStd, pred+predStd,
                    color="orange", alpha=0.12, label="Prediction ±1σ")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel(label)
    ax.grid(True)
    ax.legend()
    fig.tight_layout()

def plotResiduals(residual, std, label):
    fig, ax = plt.subplots(figsize=(12, 5))

    ax.plot(t, residual, "ko-", markersize=3, label="Innovation")
    ax.axhline(0, color="black")

    for n in (1, 2, 3):
        ax.plot(t, n*std, "--", label=f"+{n}σ")
        ax.plot(t, -n*std, "--", label=f"-{n}σ")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel(label)
    ax.grid(True)
    ax.legend()
    fig.tight_layout()

plotPosition(
    "X", data["groundTruthX"], data["GPSX"],
    data["predictedX"], data["estimatedX"],
    data["predictedStdX"], data["estimatedStdX"], "X position (m)"
)

plotPosition(
    "Y", data["groundTruthY"], data["GPSY"],
    data["predictedY"], data["estimatedY"],
    data["predictedStdY"], data["estimatedStdY"], "Y position (m)"
)

plotResiduals(data["residualX"], data["inovStdX"], "X residual (m)")
plotResiduals(data["residualY"], data["inovStdY"], "Y residual (m)")

plt.show()
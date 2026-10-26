# Multivariate Kalman Filter in 2D

## Overview

This repository implements a Multivariate Kalman Filter in a Webots simulation to study how state-estimation behaviour changes under different initial-state, process-noise, and measurement-noise assumptions.

The experiments compare predicted states and posterior estimates with the robot's simulated ground-truth position and noisy GPS measurements. The focus is on understanding the filter's behaviour—not finding one parameter setting that is optimal for every situation.

## Motivation

This project is part of my effort to strengthen the fundamentals of probabilistic robotics and state estimation before working on autonomous mobile robot (AMR) localization and SLAM.

Rather than treating the Kalman Filter as a set of equations, I wanted to observe how it combines a motion model with noisy measurements, how its uncertainty changes, and how different parameter settings affect the estimates.

## Simulation

The simulation uses a simplified differential-drive robot in Webots. The controller records the robot's ground-truth position, GPS measurements, predictions, posterior estimates, uncertainty, and innovations for plotting.

| Item | Details |
|---|---|
| Simulator | Webots R2025a |
| Robot | Simplified differential-drive robot |
| Measurement source | GPS position in the world frame |
| State vector | Position and velocity in two dimensions |
| Motion model | Linear constant-velocity model |
| Filter time step | 0.064 s |
| Python packages | NumPy and Matplotlib |

Note : GPS is used as a convenient source of position measurements for studying filtering. This setup is not intended to demonstrate a practical GPS-based localization solution for an indoor warehouse.

## Mathematical Model

The filter estimates the state

$$
\mathbf{x}_k =
\begin{bmatrix}
 x_k & v_{x,k} & y_k & v_{y,k}
\end{bmatrix}^{T}
$$

Here, \(x_k\) and \(y_k\) are position coordinates, while \(v_{x,k}\) and \(v_{y,k}\) are velocities. Velocity is not directly measured; it is inferred through the motion model and the available position measurements so, velocities in this experiment are hidden varaibles where as the positions are observable variables. The current state vector does not include heading \(theta\).

### Prediction

For a constant-velocity model,

$$
\hat{\mathbf{x}}_{k|k-1} = F\hat{\mathbf{x}}_{k-1|k-1}
$$

Here, the prediction step doesnot include the control input as there are no control inputs to the robot



$$
P_{k|k-1} = FP_{k-1|k-1}F^T + Q
$$

with the state-transition matrix

$$
F = \begin{bmatrix}
1 & \Delta t & 0 & 0\\
0 & 1 & 0 & 0\\
0 & 0 & 1 & \Delta t\\
0 & 0 & 0 & 1
\end{bmatrix}.
$$

Here, \(P\) is the state covariance, \(Q\) is the process-noise covariance, and \(Delta t\) is the time interval between updates. The implementation models process uncertainty using unmodelled acceleration.

### Measurement update

The measurement contains GPS position only:

$$
\mathbf{z}_k = \begin{bmatrix}x_{GPS} & y_{GPS}\end{bmatrix}^{T},
\qquad
H = \begin{bmatrix}
1 & 0 & 0 & 0\\
0 & 0 & 1 & 0
\end{bmatrix}.
$$

The innovation (measurement residual) is

$$
\mathbf{y}_k = \mathbf{z}_k - H\hat{\mathbf{x}}_{k|k-1}.
$$

The innovation covariance and Kalman gain are

$$
S_k = HP_{k|k-1}H^T + R,
\qquad
K_k = P_{k|k-1}H^TS_k^{-1}.
$$

The posterior state is

$$
\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + K_k\mathbf{y}_k.
$$

The implementation uses the Joseph form for the posterior covariance update:

$$
P_{k|k} = (I-K_kH)P_{k|k-1}(I-K_kH)^T + K_kRK_k^T.
$$

### Parameters to understand

- **\(P\) — initial-state covariance:** represents uncertainty in the initial state estimate. 
- **\(Q\) — process-noise covariance:** represents uncertainty in the motion model.
- **\(R\) — measurement-noise covariance:** represents uncertainty in the GPS measurements.

Think of that in this way
- If your are not certain or highly uncertain about the robot's initial state then set \(P\) to a higher value(std), and if you are sure then you can set (P\) to a lower value(std).
- So, \(P\) describes the confidence level of the robot's initial state and P converges to a constant value after a few iterations
- whereas \(Q\) adds uncertainity into our prediction.
- And \(R\) describes the uncertainity levels of the measurement, higher the \(R\) higher the noise in the measurements. 

The standard-deviation parameters in the controller are squared when constructing the diagonal entries of the covariance matrices.

## Experiments

Seven configurations are included. The low-\(R\), low-\(Q\) combination is not included in this set of experiments as the filter will perform ideally in this case.

### Initial pose known

<details>
<summary><strong>High R, High Q</strong> — open plots</summary>

**Position estimates**

![X position — known pose, High R / High Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_known/HighR_HighQ/XPlot.jpeg)

![Y position — known pose, High R / High Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_known/HighR_HighQ/YPlot.jpeg)

**Innovation residuals**

![X innovation — known pose, High R / High Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_known/HighR_HighQ/residualXPlot.jpeg)

![Y innovation — known pose, High R / High Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_known/HighR_HighQ/residualYPlot.jpeg)

</details>

<details>
<summary><strong>High R, Low Q</strong> — open plots</summary>

**Position estimates**

![X position — known pose, High R / Low Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_known/HighR_lowQ/XPlot.jpeg)

![Y position — known pose, High R / Low Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_known/HighR_lowQ/YPlot.jpeg)

**Innovation residuals**

![X innovation — known pose, High R / Low Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_known/HighR_lowQ/residualXPlot.jpeg)

![Y innovation — known pose, High R / Low Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_known/HighR_lowQ/residualYPlot.jpeg)

</details>

<details>
<summary><strong>Low R, High Q</strong> — open plots</summary>

**Position estimates**

![X position — known pose, Low R / High Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_known/LowR_HighQ/XPlot.jpeg)

![Y position — known pose, Low R / High Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_known/LowR_HighQ/YPlot.jpeg)

**Innovation residuals**

![X innovation — known pose, Low R / High Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_known/LowR_HighQ/residualXPlot.jpeg)

![Y innovation — known pose, Low R / High Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_known/LowR_HighQ/residualYPlot.jpeg)

</details>

<details>
<summary><strong>Very Low R, Very High Q</strong> — open plots</summary>

**Position estimates**

![X position — known pose, Very Low R / Very High Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_known/veryLowR_VeryHighQ/XPlot.jpeg)

![Y position — known pose, Very Low R / Very High Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_known/veryLowR_VeryHighQ/YPlot.jpeg)

**Innovation residuals**

![X innovation — known pose, Very Low R / Very High Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_known/veryLowR_VeryHighQ/residualXPlot.jpeg)

![Y innovation — known pose, Very Low R / Very High Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_known/veryLowR_VeryHighQ/residualYPlot.jpeg)

</details>

### Initial pose unknown

<details>
<summary><strong>High R, High Q</strong> — open plots</summary>

**Position estimates**

![X position — unknown pose, High R / High Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_unkown/HighR_HighQ/XPlot.jpeg)

![Y position — unknown pose, High R / High Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_unkown/HighR_HighQ/YPlot.jpeg)

**Innovation residuals**

![X innovation — unknown pose, High R / High Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_unkown/HighR_HighQ/residualXPlot.jpeg)

![Y innovation — unknown pose, High R / High Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_unkown/HighR_HighQ/residualYPlot.jpeg)

</details>

<details>
<summary><strong>High R, Low Q</strong> — open plots</summary>

**Position estimates**

![X position — unknown pose, High R / Low Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_unkown/HighR_lowQ/XPlot.jpeg)

![Y position — unknown pose, High R / Low Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_unkown/HighR_lowQ/YPlot.jpeg)

**Innovation residuals**

![X innovation — unknown pose, High R / Low Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_unkown/HighR_lowQ/residualXPlot.jpeg)

![Y innovation — unknown pose, High R / Low Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_unkown/HighR_lowQ/residualYPlot.jpeg)

</details>

<details>
<summary><strong>Low R, High Q</strong> — open plots</summary>

**Position estimates**

![X position — unknown pose, Low R / High Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_unkown/LowR_HighQ/XPlot.jpeg)

![Y position — unknown pose, Low R / High Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_unkown/LowR_HighQ/YPlot.jpeg)

**Innovation residuals**

![X innovation — unknown pose, Low R / High Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_unkown/LowR_HighQ/residualXPlot.jpeg)

![Y innovation — unknown pose, Low R / High Q](results/multivariate_kalman_filter_resultsplots/plots_Initial_pose_unkown/LowR_HighQ/residualYPlot.jpeg)

</details>

## How to explore the plots

Before opening a configuration, try to predict its behaviour from the meanings of \(P\), \(Q\), and \(R\). Then compare your expectation with the plots.

1. **Initial transient:** When the initial state estimate is incorrect, observe how the prediction and posterior estimate evolve relative to ground truth.
2. **Prediction versus posterior:** Watch how measurements correct the predicted state. Larger assumed measurement uncertainty generally reduces the influence of a measurement; larger process uncertainty generally allows the estimate to respond more to measurements.
3. **Uncertainty bands:** Compare the predicted and posterior bands. Under a Gaussian model, approximately 68.3% of values lie within |std|, 95.4% within |2std|, and 99.7% within |3std|. These percentages are reference expectations, not guarantees for a particular run.
4. **Innovation residuals:** Compare the innovation with its predicted uncertainty bounds. Persistent patterns, large residuals, or repeated excursions beyond the bounds can indicate behaviour worth investigating. A single crossing does not, by itself, establish that the filter is inconsistent.

One observation from these experiments is that, when measurements are considered uncertain and the motion model is reasonably accurate, a lower process-noise setting can produce a smoother estimate. The residual plots are included because position estimates that look similar can still have different innovation behaviour.

## Setup and execution

### Requirements

- Webots R2025a
- Python 3.12 or newer for the current controller's f-string syntax
- NumPy
- Matplotlib

Install the Python dependencies with:

```bash
python -m pip install -r requirements.txt
```

### Run the simulation

1. Open the `.wbt` world in the `worlds/` directory with Webots.
2. Verify that the world's `controller` field matches the controller directory name under `controllers/`.
3. Start the simulation and allow the controller to collect the measurements and state estimates. The controller saves `MultivariateKF_results.npz` when the run finishes.

**Controller-name check:** The supplied world file currently names the controller `controllerMultivariateKF`, while the directory tree shared for this repository names its folder `controller_multivariate_kalman_filter`. These names must match for Webots to find the controller. Rename the folder or update the world's `controller` field before running.

### Generate the plots

The plotting script expects `MultivariateKF_results.npz` to be in the same directory as `plotKalmaFilterResults.py`. From that directory, run:

```bash
python plotKalmaFilterResults.py
```

The script opens position and innovation plots for the saved run. The plots in the configuration folders are retained outputs from the parameter sweeps.

## Scope and limitations

- **Linear motion model:** The filter uses a constant-velocity model. Process noise can account for unmodelled acceleration, but it does not make the model robust to arbitrary nonlinear motion.
- **Position-only measurements:** Velocity is inferred from the motion model and successive position information.
- **No heading estimate:** Heading is not included in the current state vector.
- **Simulation data:** Results come from Webots, not a physical robot.
- **Manually selected noise settings:** The experiments vary covariance parameters manually rather than estimating them adaptively.
- **Selected configurations:** These seven runs are an illustrative comparison, not an exhaustive parameter sweep.

The uploaded world file sets the GPS `accuracy` field to `-1`, while the controller currently assumes `stdZX = stdZY = 3` when constructing \(R\). Webots documents GPS accuracy as the standard deviation of Gaussian position noise and expects a non-negative value. Verify and align the world-file setting and the assumed measurement covariance before treating the sensor noise as calibrated. See the [Webots GPS reference](https://cyberbotics.com/doc/reference/gps?version=R2025a).

The unknown-initial-pose plots were produced as a separate experimental configuration. The uploaded controller currently initializes its state estimate at zero, so reproducing those plots requires restoring the initial-state offset used for that run. The plots document the experiments; the exact per-run parameter values should be recorded if strict reproducibility is needed.

## Future work

The next stage of this learning series will explore filtering methods that support nonlinear motion and measurement models, followed by their application to robot localization and SLAM. The broader objective is to build a sound state-estimation foundation for an autonomous mobile robot.

## References

- Roger R. Labbe, [*Kalman and Bayesian Filters in Python*](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python) — an educational reference for building intuition around Kalman and Bayesian filters.
- Cyberbotics, [Webots GPS reference manual](https://cyberbotics.com/doc/reference/gps?version=R2025a).
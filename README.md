# Kalman Filter Trajectory Estimation

<p align="center">
  <img src="https://img.shields.io/badge/MATLAB-Kalman%20Filter-orange?style=for-the-badge">
  <img src="https://img.shields.io/badge/Control-State%20Estimation-blue?style=for-the-badge">
</p>

---

## Overview

This project implements a **discrete-time Kalman Filter** in MATLAB to estimate the trajectory of a moving object from noisy measurements.

The simulation:

* generates a real trajectory,
* adds Gaussian measurement noise,
* applies a Kalman Filter,
* reconstructs the original trajectory.

The filter estimates both:

* position,
* velocity,

even though only the position is directly measured.

---

## Mathematical Model

The system is modeled using a discrete state-space representation:

```text id="mfzqoo"
x(k+1) = A x(k) + w(k)
y(k)   = H x(k) + v(k)
```

Where:

| Variable | Description                         |
| -------- | ----------------------------------- |
| `x(k)`   | State vector `[position velocity]ᵀ` |
| `A`      | State transition matrix             |
| `H`      | Observation matrix                  |
| `w(k)`   | Process noise                       |
| `v(k)`   | Measurement noise                   |

State transition matrix:

```text id="48vtzx"
A = [1 dt
     0 1]
```

Observation matrix:

```text id="fc1u2n"
H = [1 0]
```

---

## Kalman Filter Algorithm

The filter operates in two stages:

### Prediction

```text id="1h6xt5"
x_pred = A x_hat
```

### Correction

```text id="6wsryh"
x_hat = x_pred + K(measurement - Hx_pred)
```

The Kalman Gain `K` automatically balances:

* model prediction,
* sensor confidence.

---

## Simulation Parameters

| Parameter             | Value |
| --------------------- | ----- |
| Time Step `dt`        | 1     |
| Process Noise `q`     | 0.1   |
| Measurement Noise `R` | 0.1   |
| Simulation Steps `N`  | 50    |

---

## Results

The simulation compares:

* true trajectory,
* noisy measurements,
* filtered trajectory.

The Kalman Filter successfully reduces noise and reconstructs the original motion.

---

## Applications

Kalman Filters are widely used in:

* aerospace guidance,
* robotics,
* GPS navigation,
* autonomous vehicles,
* radar tracking,
* sensor fusion,
* embedded systems.

---

## Technologies Used

* MATLAB
* State-Space Modeling
* Stochastic Estimation
* Control Systems

---

## Author

**Fils Elie Boungoueres**
M.Sc. Complex Systems Engineering — AM2AS
University of Bordeaux

Fields:

* Control Systems
* Embedded Systems
* Aerospace Dynamics
* Signal Processing

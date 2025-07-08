# KalmanFilterCHOP — Real-Time Multi-Channel Kalman Filtering for TouchDesigner

A fully configurable multi-channel Kalman Filter implemented in TouchDesigner using Script CHOP.

> Designed for real-time signal filtering, noise suppression, and adaptive estimation in live audio/visual systems.

---

![KalmanFilterCHOP](images/Kalman_Full_Screen.png)

---


For more information see [Wiki](https://github.com/saimgulay/TouchDesigner-Kalman-FilterCHOP/wiki/KalmanFilter-CHOP).

---


## 🔍 Overview

The Kalman Filter is a recursive algorithm for optimal estimation of dynamic systems under uncertainty. Widely used across disciplines, it combines prediction and correction to filter noisy measurements and estimate hidden variables.

Its applications span from aerospace navigation, robotics, autonomous vehicles, and signal processing, to finance, neuroscience, and even interactive art installations. Whether used for sensor fusion, object tracking, motion capture, or predictive modelling, the Kalman Filter remains a cornerstone in real-time data interpretation.

This document provides a domain-wise breakdown of its application in TouchDesigner in a straightforward way.

---


## Features

- Multi-channel, n-dimensional Kalman filtering
- Adjustable Process & Measurement Noise (Q, R)
- Custom transition (A) and measurement (H) matrices via DATs
- Filter per sample or per frame
- Pass count multiplier
- Internal trail history with CHOP viewer support
- Parameters: Cutoff frequency, slope limiting, speed compensation
- Reset functionality, history buffer, and live-tweaking support

---

## Installation

1. Drag & drop `kalmanfilter1.tox` into your TouchDesigner network.
2. Connect a CHOP input (e.g. Noise, LFO, OSC).
3. Configure parameters via the custom parameter page.
4. Optionally, attach a Trail CHOP to visualise output over time.

---



# KalmanFilterCHOP: Real-Time Adaptive Filtering for TouchDesigner

## Introduction

Real-time data streams—especially in creative coding environments like TouchDesigner—are rarely clean. Whether sourced from sensors, audio, or user input, they often carry noise, latency, or unpredictable spikes. To deal with this chaos, I implemented a fully configurable multi-channel Kalman Filter as a Script CHOP, under the name `kalmanfilter1`.

---

## Motivation

TouchDesigner lacks a built-in, general-purpose, multi-dimensional Kalman filter. I wanted something that can be:

- Drop-in for any multi-channel CHOP input  
- Visual, with full trail history inside the Script CHOP  
- Tunable, with real-time parameter control  
- And most importantly: stable, robust, and fast  

This wasn’t just about filtering—it was about making signal behavior visible and expressive in an environment meant for real-time human interaction.

---

## Justification

Compared to other Filter CHOPs, Kalman filtering offers:

- A probabilistic model: not just smoothing, but estimation  
- Configurable trust in system dynamics (`Q`) vs. measurements (`R`)  
- Support for matrix-based modeling (`A`, `H`), and temporal adaptation (`Δt`)  
- With some care: speed/slope control, multi-pass refinement, and per-sample filtering  

All of that is now implemented inside `kalmanfilter1`, with a compact parameter UI and an internal state machine that handles reset, reconfiguration, and overflow-safe history tracking.

The n-dimensional Kalman filter is a recursive Bayesian estimator widely used for state estimation in linear dynamical systems with Gaussian noise. Its standard formulation assumes a process model of the form:

  **xₖ = A xₖ₋₁ + w, w ~ 𝒩(0, Q)**  
  **zₖ = H xₖ + v, v ~ 𝒩(0, R)**

where **xₖ** is the hidden state at time *k*, **zₖ** is the observed measurement, **A** is the state transition matrix, **H** is the observation matrix, and **Q**, **R** are the process and observation noise covariances, respectively. This formulation was first introduced by Rudolf E. Kalman in his seminal paper *“A New Approach to Linear Filtering and Prediction Problems”* ([Kalman, 1960](https://doi.org/10.1115/1.3662552)), and has since become a foundational tool in control theory, navigation, signal processing, and robotics.

In practical applications, numerical instability can arise during matrix inversion (especially in the covariance update step). To address this, various robust Kalman filtering approaches have been proposed, such as those discussed by Cipra & Romera (1991) in *“Robust Kalman Filter and its Application in Time Series Analysis”*, where inversion fallback strategies and robustified updates are presented for handling ill-conditioned or uncertain models.


---

## Tests

I conducted three main tests:

1. **Sine wave test** – classic noisy input with smooth oscillations  
2. **Step response** – abrupt signal jump; tested filter latency and overshoot  
3. **Ramp input** – slow, linear growth; used to examine slope stability and lag  

Each case was evaluated by comparing:

- Raw measurements  
- True ground-truth signal  
- Kalman-filtered output  

We computed RMSE (Root Mean Square Error) between the filtered signal and the true one. Results showed low error, visibly smooth outputs, and preserved dynamics.

---

## Evaluation

The filter tracked both abrupt and slow changes effectively.

### Key outcomes:

- **Step signal:** quick response without overshooting  
- **Ramp signal:** lag-free tracking with strong jitter suppression  
- **Sine:** very low noise bleed-through, even with moderate Q/R  

Additionally, the Script CHOP’s internal trail buffer allowed direct visual feedback without needing external Trail CHOPs—essential for rapid prototyping.

---

## Future Development

Planned improvements:

- Dynamic Q/R adjustment based on signal variance (adaptive Kalman)  
- Optional velocity & acceleration tracking (state vector extension)  
- Multi-filter pipelines (e.g., pre-smoother + Kalman)  
- Exportable presets and OSC/MIDI bindings for live performance  

Eventually, this logic might be rewritten as a C++ CHOP for maximum CPU efficiency—but `kalmanfilter1` already delivers clear, fast, and flexible results.

---

## Appendix

### Ramp Input Test  
![Ramp Input](images/ramp_input.png)

### Kalman Filter Ramp Response  
![Ramp Response](images/RampResponse.png)

### Sine Wave Test  
![Sine](images/sine_wave.png)

### Kalman Filter Performance on Noisy Sine Wave  
![SineWaveTest](images/SineWaveTest.png)

### Step Response Test  
![Step](images/step_response.png)

### Kalman Filter Step Response  
![StepResponse](images/StepResponse.png)


## 📄 License

MIT License – see [LICENSE](./LICENSE) for details.


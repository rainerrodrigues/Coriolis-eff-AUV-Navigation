# Trans-Oceanic AUV Navigation: EKF & Coriolis Compensation

**An autonomous underwater vehicle (AUV) simulation demonstrating trans-oceanic navigation through the Indian Ocean, utilizing an Extended Kalman Filter (EKF) to compensate for Earth's rotation (Coriolis $\beta$-plane) and chaotic monsoon ocean currents.**

![Project Status](https://img.shields.io/badge/Status-Complete-success)
![Language](https://img.shields.io/badge/Language-Julia-9558B2)
![Framework](https://img.shields.io/badge/Framework-Oceananigans.jl-blue)

*(Drop a GIF of your CairoMakie animation here)*

## 🌊 Project Overview
Navigating a small robotic craft across 5,000 kilometers of open ocean (Mumbai to Madagascar) introduces massive physical complexities. A robot relying solely on dead-reckoning will be blown hundreds of kilometers off course by massive ocean gyres and the rotation of the Earth.

This project simulates a long-endurance AUV crossing the Indian Ocean. It proves that by using an **Extended Kalman Filter**, the robot can filter out noisy sensor data, estimate its true position in a chaotic fluid environment, and dynamically calculate the exact thrust vector required to counteract planetary physics and reach its target.

## 🧠 The Architecture

### 1. The Physics: $\beta$-Plane Coriolis Drift
As the AUV travels South toward the equator, the rotational velocity of the Earth changes. This induces a non-linear lateral drift. The simulation uses a $\beta$-plane approximation:
$f \approx f_0 + \beta y$
The AUV's control algorithm dynamically predicts this acceleration and alters its yaw heading to zero out the cross-track error.

### 2. The Environment: Real-World Monsoon Forcing
Instead of a static test pool, the simulation environment is built in `Oceananigans.jl`. The surface boundary conditions are driven by real-world Southwest Monsoon wind-stress data (via NetCDF climate files), naturally spinning up a chaotic, turbulent western boundary current that the AUV must fight through.

### 3. The "Brain": Extended Kalman Filter (EKF)
Because the $\beta$-plane physics are non-linear (dependent on the $Y$-position), a standard Kalman Filter fails. This project derives a 4x4 Jacobian matrix to linearize the physics at every timestep. 
The EKF successfully:
* Ingests highly noisy, simulated ADCP and IMU data.
* Filters out the "process noise" of the turbulent ocean gyre.
* Outputs a smooth, accurate state estimation for the navigation controller.

## 🚀 Quick Start

### Prerequisites
You will need Julia installed, along with the following packages:
```julia
using Pkg
Pkg.add(["Oceananigans", "CairoMakie", "NCDatasets", "Interpolations"])
```
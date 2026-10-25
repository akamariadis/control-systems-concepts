# 1D Cart PID Feedback Controller Simulation

A Python simulation of a PID feedback loop, controlling a second order dynamical plant (1D cart with viscous friction). Built from first principles using Runge-Kutta 4th Order (RK4) numerical integration and discrete time anti-windup control.

![Build Status](https://img.shields.io/badge/build-passing-brightgreen)
![Python Version](https://img.shields.io/badge/python-3.9%2B-blue)
![License](https://img.shields.io/badge/license-MIT-green)

---

## Table of Contents
- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Mathematical Formulation](#mathematical-formulation)
  - [Plant Dynamics (1D Cart)](#1-plant-dynamics-1d-cart)
  - [Continuous PID Controller](#2-continuous-pid-controller)
  - [Discrete-Time Controller Realization](#3-discrete-time-controller-realization)
  - [Actuator Saturation & Anti-Windup](#4-actuator-saturation--anti-windup)
  - [Numerical Integration (RK4)](#5-numerical-integration-rk4)
- [Running Simulations](#running-simulations)
  - [Step Response Analysis](#step-response-analysis)
  - [Tuning Profile Comparison](#tuning-profile-comparison)
- [Unit Testing](#unit-testing)

---

## Overview

Controlling physical systems requires continuous error mitigation under real-world constraints such as physical inertia, viscous air/contact damping, and finite actuator effort. 

This repository models:
1. **The Physical Plant:** A translational mass-damper system governed by Newton's second law.
2. **The Controller:** A discrete-time PID algorithm with numerical derivative-kick mitigation and dynamic anti-windup clamping.
3. **The Numerical Engine:** A 4th-order Runge-Kutta integration solver avoiding the truncation instability common in single-step Euler approximation.

---

## System Architecture

```text
                  +-----------------------------------------------+
                  |                   Controller                  |
                  |                                               |
  r(t)        e(t)|   +----+     u_p(t)                           |
Setpoint --->(+)--+-->| Kp |-----------------------(+)            |
              ^ -     +----+                        |             |  u_sat(t)    +-------------+     x(t)
              |   |                                 |             |  Force (N)   | Plant       |  Position (m)
              |   |   +----+  Integrator   u_i(t)   |             +------------->| 1D Cart     |--------+--->
              |   +-->| Ki |--[Anti-Windup]-------->(+)--->[ Clamp ]             | m*x'' + b*x'|        |
              |   |   +----+                        |      [Limits ]             +-------------+        |
              |   |                                 |                                                   |
              |   |   +----+  Derivative   u_d(t)   |                                                   |
              |   +-->| Kd |--[de(t)/dt]----------->(+)                                                 |
              |       +----+                                                                            |
              |                                                                                         |
              +-----------------------------------------------------------------------------------------+
                                                  Feedback (Sensor)

```

---

## Mathematical Formulation

### 1. Plant Dynamics (1D Cart)

The physical system models a 1D cart of mass $m$ experiencing linear viscous damping $b$, driven by an external horizontal actuator force $F(t)$. By Newton's Second Law of Motion, the governing second-order differential equation is:

$$
m\ddot{x}(t) + b\dot{x}(t) = F(t)
$$

Where:
* $x(t)$ is the cart position ($\text{m}$)
* $\dot{x}(t) = v(t)$ is the cart velocity ($\text{m/s}$)
* $\ddot{x}(t) = a(t)$ is the cart acceleration ($\text{m/s}^2$)
* $m$ is the payload mass ($\text{kg}$)
* $b$ is the viscous damping coefficient ($\text{N}\cdot\text{s/m}$)
* $F(t) = u_{\text{sat}}(t)$ is the saturated control force applied to the cart ($\text{N}$)

**State-Space Representation**

To solve the dynamics numerically using standard ODE solvers, the second-order differential equation is reduced to a system of two coupled first-order differential equations. Defining the state vector as $\mathbf{x}(t) = [x(t), \dot{x}(t)]^T$, the continuous-time state-space derivative is:

$$
\dot{\mathbf{x}}(t) = \mathbf{f}(\mathbf{x}(t), u_{\text{sat}}(t)) = \begin{bmatrix} \dot{x}(t) \\ \ddot{x}(t) \end{bmatrix} = \begin{bmatrix} \dot{x}(t) \\ \frac{1}{m}\left(u_{\text{sat}}(t) - b\dot{x}(t)\right) \end{bmatrix}
$$

In linear matrix form ($\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}u$):

$$
\begin{bmatrix} \dot{x} \\ \ddot{x} \end{bmatrix} = \begin{bmatrix} 0 & 1 \\ 0 & -\frac{b}{m} \end{bmatrix} \begin{bmatrix} x \\ \dot{x} \end{bmatrix} + \begin{bmatrix} 0 \\ \frac{1}{m} \end{bmatrix} u_{\text{sat}}(t)
$$

**Laplace Transfer Function**

Taking the Laplace transform assuming zero initial conditions yields the open-loop plant transfer function $G(s)$ from input force $F(s)$ to output position $X(s)$:

$$
G(s) = \frac{X(s)}{F(s)} = \frac{1}{ms^2 + bs} = \frac{1}{s(ms + b)}
$$

---

### 2. Continuous PID Controller

The controller seeks to drive the tracking error $e(t)$ between the target reference setpoint $r(t)$ and the measured cart position $x(t)$ to zero:

$$
e(t) = r(t) - x(t)
$$

The ideal parallel continuous-time PID control law computes the unconstrained control effort $u(t)$ as the sum of Proportional ($u_p$), Integral ($u_i$), and Derivative ($u_d$) terms:

$$
u(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau)\,d\tau + K_d \frac{de(t)}{dt}
$$

Where:
* $K_p$ is the Proportional gain ($\text{N/m}$), providing restoring force proportional to current error.
* $K_i$ is the Integral gain ($\text{N}/(\text{m}\cdot\text{s})$), eliminating steady-state offset by accumulating past error.
* $K_d$ is the Derivative gain ($\text{N}\cdot\text{s/m}$), injecting artificial damping based on the rate of error change.

**Derivative-on-Measurement (Derivative Kick Mitigation)**

Step changes in the setpoint $r(t)$ cause instantaneous discontinuities in $e(t)$, resulting in an impulse spike in the derivative term known as *derivative kick*. Since $\frac{dr(t)}{dt} = 0$ for a constant step setpoint, the derivative action can be computed directly on the negative velocity of the plant:

$$
u_d(t) = -K_d \frac{dx(t)}{dt}
$$

---

### 3. Discrete-Time Controller Realization

For digital simulation over a fixed sampling period $\Delta t = t_k - t_{k-1}$ at discrete step index $k \in \{0, 1, 2, \dots, N\}$, the continuous operators are approximated via finite difference equations:

**Tracking Error:**

$$
e_k = r_k - x_k
$$

**Proportional Term:**

$$
u_{p,k} = K_p e_k
$$

**Integral Term (Forward Euler Approximation):**

$$
u_{i,k} = u_{i,k-1} + K_i e_k \Delta t
$$

**Derivative Term (Backward Finite Difference):**

$$
u_{d,k} = K_d \left( \frac{e_k - e_{k-1}}{\Delta t} \right)
$$

**Total Unconstrained Control Output:**

$$
u_k = u_{p,k} + u_{i,k} + u_{d,k}
$$

---

### 4. Actuator Saturation & Anti-Windup

Physical motors and actuators are bounded by maximum hardware force limits $\pm U_{\max}$ (where $U_{\min} = -U_{\max}$). The raw controller command $u_k$ passes through a non-linear saturation block before reaching the plant:

$$
u_{\text{sat},k} = \max\left(U_{\min}, \, \min\left(U_{\max}, \, u_k\right)\right)
$$

Which expands piecewise to:

$$
u_{\text{sat},k} = \begin{cases} U_{\max} & \text{if } u_k > U_{\max} \\ u_k & \text{if } U_{\min} \le u_k \le U_{\max} \\ U_{\min} & \text{if } u_k < U_{\min} \end{cases}
$$

**Conditional Integration (Anti-Windup Clamping)**

When $u_k$ exceeds the physical limits ($u_k \neq u_{\text{sat},k}$), the actuator saturates and the feedback loop effectively opens. Without mitigation, persistent tracking error causes the integral accumulator $u_{i,k}$ to wind up to massive values, inducing severe overshoot and lag.

To prevent windup, conditional clamping freezes the integral update whenever the actuator is saturated and the current error pushes deeper into saturation:

$$
u_{i,k} = \begin{cases} u_{i,k-1} & \text{if } (u_k \neq u_{\text{sat},k}) \text{ and } (\text{sgn}(e_k) = \text{sgn}(u_k)) \\ u_{i,k-1} + K_i e_k \Delta t & \text{otherwise} \end{cases}
$$

Alternatively, under **Static Integrator Clamping**, the accumulated integral term is directly bounded to the actuator limits:

$$
u_{i,k} = \max\left(U_{\min}, \, \min\left(U_{\max}, \, u_{i,k-1} + K_i e_k \Delta t\right)\right)
$$

---

### 5. Numerical Integration (RK4)

Rather than relying on first-order Forward Euler integration, which accumulates local truncation error of $\mathcal{O}(\Delta t^2)$ and injects artificial energy into oscillatory systems, the plant dynamics are propagated using the classical **4th-Order Runge-Kutta (RK4)** solver.

Assuming a Zero-Order Hold (ZOH) on the saturated control input $u_{\text{sat},k}$ across the time interval $[t_k, t_k + \Delta t]$, RK4 evaluates the state derivative $\mathbf{f}(\mathbf{x}, u_{\text{sat}})$ at four intermediate trial points:

$$
\mathbf{k}_1 = \mathbf{f}\left(\mathbf{x}_k, \, u_{\text{sat},k}\right)
$$

$$
\mathbf{k}_2 = \mathbf{f}\left(\mathbf{x}_k + \frac{\Delta t}{2}\mathbf{k}_1, \, u_{\text{sat},k}\right)
$$

$$
\mathbf{k}_3 = \mathbf{f}\left(\mathbf{x}_k + \frac{\Delta t}{2}\mathbf{k}_2, \, u_{\text{sat},k}\right)
$$

$$
\mathbf{k}_4 = \mathbf{f}\left(\mathbf{x}_k + \Delta t\mathbf{k}_3, \, u_{\text{sat},k}\right)
$$

The state vector at the next discrete timestep $t_{k+1} = t_k + \Delta t$ is computed via the weighted average of the four slopes:

$$
\mathbf{x}_{k+1} = \mathbf{x}_k + \frac{\Delta t}{6}\left(\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4\right)
$$

This yields a local truncation error of $\mathcal{O}(\Delta t^5)$ and a global accumulated error of $\mathcal{O}(\Delta t^4)$, ensuring accurate continuous-time physics emulation even under stiff PID gains.

---

## Running Simulations

Ensure you have Python 3.9+ installed along with `numpy` and `matplotlib`. Install requirements using:

```bash
pip install -r requirements.txt
```

### Step Response Analysis
To execute a standard step response simulation from $x = 0$ to a target position of $x = 10$, run the main script:

```bash
python main.py --mode step
```

This generates a plot showing the cart's position, velocity, and applied control force over time, explicitly illustrating the actuator saturation limits and settling time.

### Tuning Profile Comparison
To visualize how different PID constants affect the system, run the comparison mode:

```bash
python main.py --mode compare
```

This generates an overlaid step-response chart demonstrating three classical tuning regimes:
* **Overdamped:** High $K_d$, sluggish response with zero overshoot.
* **Critically Damped:** Optimal balance, fastest approach without oscillation.
* **Underdamped:** High $K_p$ and $K_i$, aggressive response with visible ringing and overshoot.

---

## Unit Testing

The repository uses `pytest` to validate numerical accuracy and controller logic. The test suite verifies:
* **Zero-State Stability:** Zero error yields zero control output.
* **Windup Mitigation:** Integrator values correctly clamp when simulated bounds are exceeded.
* **RK4 Precision:** Physics engine matches analytical solutions for unforced decay under viscous friction.

Run the test suite via:

```bash
pytest tests/
```

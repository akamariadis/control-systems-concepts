# 1D Cart PID Feedback Controller Simulation

# [Currently in construction...]

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

## Mathematical Formulation

### 1. Plant Dynamics (1D Cart)
The physical system models a 1D cart of mass $m$ experiencing linear viscous damping $b$, driven by an external horizontal actuator force $F(t)$. By Newton's Second Law of Motion ($\sum F = ma$), the governing second-order differential equation is:

$$m\ddot{x}(t) + b\dot{x}(t) = F(t)$$

Where:
* $x(t)$ is the cart position ($\text{m}$)
* $\dot{x}(t) = v(t)$ is the cart velocity ($\text{m/s}$)
* $\ddot{x}(t) = a(t)$ is the cart acceleration ($\text{m/s}^2$)
* $m$ is the payload mass ($\text{kg}$)
* $b$ is the viscous damping coefficient ($\text{N}\cdot\text{s/m}$)
* $F(t) = u_{\text{sat}}(t)$ is the saturated control force applied to the cart ($\text{N}$)

**State-Space Representation:**
To solve the dynamics numerically using standard ODE solvers, the second-order differential equation is reduced to a system of two coupled first-order differential equations. Defining the state vector as $\mathbf{x}(t) = \begin{bmatrix} x_1(t) \\ x_2(t) \end{bmatrix} = \begin{bmatrix} x(t) \\ \dot{x}(t) \end{bmatrix}$, the continuous-time state-space model is:

$$\dot{\mathbf{x}}(t) = \mathbf{f}(\mathbf{x}(t), u_{\text{sat}}(t)) = \begin{bmatrix} \dot{x}_1(t) \\ \dot{x}_2(t) \end{bmatrix} = \begin{bmatrix} x_2(t) \\ \frac{1}{m}\left(u_{\text{sat}}(t) - b x_2(t)\right) \end{bmatrix}$$

In linear matrix form ($\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}u$):

$$\begin{bmatrix} \dot{x} \\ \ddot{x} \end{bmatrix} = \begin{bmatrix} 0 & 1 \\ 0 & -\frac{b}{m} \end{bmatrix} \begin{bmatrix} x \\ \dot{x} \end{bmatrix} + \begin{bmatrix} 0 \\ \frac{1}{m} \end{bmatrix} u_{\text{sat}}(t)$$

**Laplace Transfer Function:**
Taking the Laplace transform assuming zero initial conditions ($x(0) = 0, \dot{x}(0) = 0$) yields the open-loop plant transfer function $G(s)$ from input force $F(s)$ to output position $X(s)$:

$$G(s) = \frac{X(s)}{F(s)} = \frac{1}{ms^2 + bs} = \frac{1}{s(ms + b)}$$

---

### 2. Continuous PID Controller
The controller seeks to drive the tracking error $e(t)$ between the target reference setpoint $r(t)$ and the measured cart position $x(t)$ to zero:

$$e(t) = r(t) - x(t)$$

The ideal parallel continuous-time PID control law computes the unconstrained control effort $u(t)$ as the sum of Proportional ($u_p$), Integral ($u_i$), and Derivative ($u_d$) terms:

$$u(t) = u_p(t) + u_i(t) + u_d(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau)\,d\tau + K_d \frac{de(t)}{dt}$$

Where:
* $K_p$ is the Proportional gain ($\text{N/m}$), providing stiffness proportional to current error.
* $K_i$ is the Integral gain ($\text{N}/(\text{m}\cdot\text{s})$), eliminating steady-state offset by accumulating past error.
* $K_d$ is the Derivative gain ($\text{N}\cdot\text{s/m}$), injecting artificial damping based on the rate of error change.

**Derivative-on-Measurement (Derivative Kick Mitigation):**
Step changes in the setpoint $r(t)$ cause instantaneous discontinuities in $e(t)$, resulting in an infinite impulse ($\frac{dr}{dt} \to \infty$) in the derivative term known as *derivative kick*. Since $\frac{de(t)}{dt} = \frac{dr(t)}{dt} - \frac{dx(t)}{dt}$ and $\frac{dr(t)}{dt} = 0$ for a constant step setpoint, the derivative action can optionally be computed directly on the negative rate of plant position:

$$u_d(t) = -K_d \frac{dx(t)}{dt}$$

---

### 3. Discrete-Time Controller Realization
For digital simulation over a fixed sampling period $\Delta t = t_k - t_{k-1}$ at discrete step index $k \in \{0, 1, 2, \dots, N\}$, the continuous operators are approximated via finite difference equations:

1. **Tracking Error:**
   $$e[k] = r[k] - x[k]$$

2. **Proportional Term:**
   $$u_p[k] = K_p \, e[k]$$

3. **Integral Term (Forward Euler / Rectangular Approximation):**
   $$I[k] = I[k-1] + e[k]\Delta t$$
   $$u_i[k] = K_i \, I[k] = u_i[k-1] + K_i \, e[k]\Delta t$$

4. **Derivative Term (Backward Finite Difference):**
   $$u_d[k] = K_d \left( \frac{e[k] - e[k-1]}{\Delta t} \right)$$
   *(Or $u_d[k] = -K_d \left(\frac{x[k] - x[k-1]}{\Delta t}\right)$ when derivative-kick mitigation is enabled).*

5. **Total Unconstrained Control Output:**
   $$u[k] = u_p[k] + u_i[k] + u_d[k]$$

---

### 4. Actuator Saturation & Anti-Windup
Physical motors and actuators are bounded by maximum hardware force limits $\pm U_{\max}$ (where $U_{\min} = -U_{\max}$). The raw controller command $u[k]$ passes through a non-linear saturation block before reaching the plant:

$$u_{\text{sat}}[k] = \text{clip}(u[k], U_{\min}, U_{\max}) = \begin{cases} U_{\max} & \text{if } u[k] > U_{\max} \\ u[k] & \text{if } U_{\min} \le u[k] \le U_{\max} \\ U_{\min} & \text{if } u[k] < U_{\min} \end{cases}$$

**Conditional Integration (Anti-Windup Clamping):**
When $u[k]$ exceeds the physical limits ($u[k] \neq u_{\text{sat}}[k]$), the actuator saturates and the feedback loop effectively opens. Without mitigation, persistent tracking error causes the integral accumulator $u_i[k]$ to wind up to massive values, inducing severe overshoot and lag before the integrator can unwind once the cart reaches the setpoint.

To prevent windup, conditional integration freezes the integral update whenever the actuator is saturated **and** the current error shares the same algebraic sign as the unsaturated control signal (pushing deeper into saturation):

$$u_i[k] = \begin{cases} u_i[k-1] & \text{if } \left(u[k] \neq u_{\text{sat}}[k]\right) \land \left(\text{sgn}(e[k]) = \text{sgn}(u[k])\right) \\ u_i[k-1] + K_i \, e[k]\Delta t & \text{otherwise} \end{cases}$$

Alternatively, if **Dynamic Integrator Clamping** is used, the integral term is directly bounded between the remaining actuator headroom:

$$u_i[k] = \text{clip}\left(u_i[k-1] + K_i e[k]\Delta t, \; U_{\min}, \; U_{\max}\right)$$

---

### 5. Numerical Integration (RK4)
Rather than relying on first-order Forward Euler integration ($\mathbf{x}_{k+1} = \mathbf{x}_k + \Delta t \mathbf{f}(\mathbf{x}_k, u_k)$), which accumulates local truncation error of $\mathcal{O}(\Delta t^2)$ and injects artificial energy into oscillatory systems, the plant dynamics are propagated using the classical **4th-Order Runge-Kutta (RK4)** solver.

Assuming a Zero-Order Hold (ZOH) on the saturated control input $u_{\text{sat}}[k]$ across the time interval $[t_k, t_k + \Delta t]$, RK4 evaluates the state derivative $\mathbf{f}(\mathbf{x}, u_{\text{sat}})$ at four intermediate trial points:

$$\mathbf{k}_1 = \mathbf{f}\left(\mathbf{x}_k, \, u_{\text{sat}}[k]\right)$$

$$\mathbf{k}_2 = \mathbf{f}\left(\mathbf{x}_k + \frac{\Delta t}{2}\mathbf{k}_1, \, u_{\text{sat}}[k]\right)$$

$$\mathbf{k}_3 = \mathbf{f}\left(\mathbf{x}_k + \frac{\Delta t}{2}\mathbf{k}_2, \, u_{\text{sat}}[k]\right)$$

$$\mathbf{k}_4 = \mathbf{f}\left(\mathbf{x}_k + \Delta t\mathbf{k}_3, \, u_{\text{sat}}[k]\right)$$

The state vector at the next discrete timestep $t_{k+1} = t_k + \Delta t$ is computed via the weighted Simpson's rule average of the four slopes:

$$\mathbf{x}_{k+1} = \mathbf{x}_k + \frac{\Delta t}{6}\left(\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4\right)$$

This yields a local truncation error of $\mathcal{O}(\Delta t^5)$ and a global accumulated error of $\mathcal{O}(\Delta t^4)$, ensuring accurate continuous-time physics emulation even under stiff PID gains.

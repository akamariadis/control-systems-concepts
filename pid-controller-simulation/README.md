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

# Preview Control Methods Summary

## Overview

This file summarizes the three preview control implementations for the Cart-Table / LIPM walking pattern generator in `preview_control_openloop.py`.

---

## Methods

### 1. `batch_optimization_control`

Solves a single open-loop quadratic program over the **entire horizon**.

- **Decision variable**: full control sequence `U = [u_0, ..., u_{K-1}]`
- **Cost**: `sum Q*(z_ref_k - z_k)^2 + R*u_k^2`
- **Pros**: globally optimal for the given reference; smallest tracking error
- **Cons**: not feasible for real-time online control when horizon is long

### 2. `receding_horizon_control`

MPC-style rolling optimization. At each time step, solve a smaller batch QP over a preview window of length `N`, then apply only the first control input.

- **Decision variable**: `U = [u_k, ..., u_{k+N-1}]`
- **Pros**: online feasible; adapts to state updates
- **Cons**: slightly higher tracking error than full batch due to shorter lookahead at each step

### 3. `riccati_preview_control`

Classic preview control using pre-computed steady-state gains from the Discrete Algebraic Riccati Equation (DARE).

- **Control law**: `u_k = -K @ x_k + sum_{j=1}^{N} f_j * z_ref_{k+j}`
- **Pros**: extremely fast online — no QP solver needed; constant-time per step
- **Cons**: slightly higher tracking error than batch QP; assumes LTI dynamics

---

## Validation Results (Straight-Walking Test Case)

| Method | ZMP Tracking RMS | Max Jerk |
|--------|-----------------:|---------:|
| `batch` | **5.814 mm** | 24.999 m/s³ |
| `receding_horizon` | 5.969 mm | 23.210 m/s³ |
| `riccati` | 9.819 mm | 22.905 m/s³ |

**Parameters used**
- `dt = 0.01 s`
- `z_c = 0.8 m`
- `g = 9.81 m/s²`
- `Q = 1.0`
- `R = 1e-6`
- Preview horizon `N = 160` (1.6 s)
- Step length `S = 0.2 m`
- Single support `T_single = 0.6 s`
- Double support `T_double = 0.2 s`

---

## Key Takeaways

- **Batch QP** is the benchmark — best tracking, but offline-only.
- **Receding horizon** gives a good middle ground between real-time feasibility and accuracy.
- **Riccati / DARE** is the classic biped walking solution: pre-compute `K` and `f_j` once, then run the control law in real time with minimal computation.

---

## Files

- `preview_control_openloop.py` — implementations of all three methods

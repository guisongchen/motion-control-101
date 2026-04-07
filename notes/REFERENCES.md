# Bipedal Balance Control - References & Solution Notes

**Date**: 2026-04-06  
**Project**: Phase 1.1 - PyBullet 2D Bipedal Balance  
**Status**: Reference collection for control strategy improvement

---

## Classic Foundation Papers

### 1. Zero-Moment Point (ZMP) Theory
**"Zero-Moment Point — Thirty Five Years of Its Life"**  
- Authors: Miomir Vukobratović, Branislav Borovac
- Year: 2004
- **Why read**: The definitive ZMP tutorial for bipedal balance
- **Key concept**: ZMP inside support polygon = dynamic stability
- **Application**: Use ZMP to determine if robot is statically stable

### 2. Preview Control for Walking
**"Biped Walking Pattern Generation by using Preview Control of Zero-Moment Point"**  
- Authors: Shuuji Kajita, Fumio Kanehiro, et al.
- Year: 2003
- **Why read**: Introduces preview control approach widely used in humanoid walking
- **Key concept**: Linear Inverted Pendulum Model (LIPM) simplifies CoM dynamics
- **Application**: Pre-calculate CoM trajectory to keep ZMP within support polygon

### 3. Virtual Model Control
**"Virtual Model Control: A Behavorial Approach for Bipedal Robotics"**  
- Author: Jerry Pratt
- Year: 1995
- **Why read**: Practical approach to decouple CoM and height control
- **Key concept**: Treat robot as virtual spring-damper system
- **Application**: Decouple height/attitude coupling in your controller

---

## Practical Implementation References

### 4. PyBullet Official Examples
**Location**: `examples/pybullet/examples/humanoid.py` (in PyBullet install directory)
- **What it shows**: Working torque control with explicit Jacobian calculations
- **Key techniques**:
  - Using `calculateJacobian()` for CoM control
  - Torque control with contact force constraints
  - Numerical stability settings

### 5. PyBullet QuickStart Guide
**URL**: https://pybullet.org/wordpress/index.php/quickstart-guide/
- **Relevant sections**:
  - Contact dynamics and solver parameters
  - `numSolverIterations` and `numSubSteps` for stability
  - Joint control modes comparison

---

## Key Concepts for Current Problem

Based on project `LESSONS_LEARNED.md`, these concepts address specific issues:

| Issue | Solution Approach | Primary Reference |
|-------|------------------|-------------------|
| Initial tilt on landing (5-20° forward) | Pre-stabilization + initial pose optimization | Kajita 2003 (preview control) |
| Position vs Torque control instability | Use POSITION_CONTROL with max force limits | PyBullet docs |
| CoM/height control coupling | Decouple via Virtual Model Control | Pratt 1995 |
| Contact instability / NaN | Increase solver iterations, use sub-steps | PyBullet QuickStart |
| Ankle torque insufficient for correction | Use hip/ankle coordination (ankle strategy) | Vukobratović 2004 |

---

## Suggested Reading Order

1. **Start with**: Kajita et al. (2003) — most practical for implementation
2. **Then**: Vukobratović & Borovac (2004) — theoretical foundation
3. **Reference**: PyBullet's `humanoid.py` — working code patterns
4. **If needed**: Pratt (1995) — for advanced decoupling techniques

---

## Core Equations to Implement

### Linear Inverted Pendulum Model (LIPM)
```
CoM dynamics: ẍ = (g/z) * x
where: g = gravity, z = constant CoM height, x = horizontal position
```

### ZMP Equation
```
zmp_x = CoM_x - (z_c / g) * CoM_ẍ
where: z_c = CoM height, g = gravity

Stability condition: zmp_x must be within support polygon (foot contact area)
```

### PD Control with Saturation (Practical Implementation)
```python
# Joint target = base_pose + compensation
target_angle = base_angle + kp * error + kd * error_dot
target_angle = clip(target_angle, min_limit, max_limit)
```

---

## Implementation Checklist

- [ ] Read Kajita 2003 for preview control basics
- [ ] Study PyBullet humanoid.py for Jacobian usage
- [ ] Implement CoM calculation from joint states
- [ ] Add ZMP monitoring (for analysis, not direct control)
- [ ] Test with increased solver iterations (50-100)
- [ ] Try position control with force limits before torque control

---

## Related Project Files

| File | Description |
|------|-------------|
| `biped_balance.py` | Baseline PD implementation |
| `biped_balance_working.py` | Coordinate control attempt (had height coupling issue) |
| `LESSONS_LEARNED.md` | Detailed failure analysis and attempted strategies |

---

## Additional Resources

- **Course**: MIT 6.832 (Underactuated Robotics) — lecture notes on bipedal walking
- **Book**: "Humanoid Robotics" by Shuuji Kajita et al. (2014)
- **Simulator comparison**: PyBullet vs MuJoCo contact dynamics

---

*Created*: 2026-04-06  
*Last updated*: 2026-04-06

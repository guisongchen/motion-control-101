# Control Flow Diagram — `main.py`

## High-Level Architecture

```mermaid
flowchart TB
    subgraph Init["Initialization"]
        I1["RobotModel + MuJoCo setup"]
        I2["StateEstimator"]
        I3["CentroidalMPC"]
        I4["WholeBodyController<br/>wbc / wbc_ss / wbc_ss_with_swing"]
        I5["Load optimized single-support pose"]
        I6["Resolve corner-patch local contact positions"]
        I7["Set MPC reference (x_ref, u_ref)"]
        I8["PhaseState = INIT_SETTLE"]
    end

    subgraph Loop["Main Simulation Loop (each step)"]
        direction TB

        L1["t = step * DT_SIM<br/>estimator.update()"]
        L2["update_phase_machine()<br/>check phase transitions"]
        L3{"Phase changed?"}
        L4["On SINGLE_SUPPORT entry:<br/>override WBC gains<br/>re-init contact state"]

        L5["Compute load_shift_metrics"]
        L6["Filter support point (if locked)"]
        L7["update_single_support_establishment()"]
        L8["compute_phase_com_target() -> c_ref"]
        L9["MPC set_reference()"]

        L10{"Phase == SINGLE_SUPPORT?"}
        L11["run_single_support_control()<br/>MPC + WBC solve<br/>return f_ref, tau, active_solver"]
        L12["f_ref = u_ref<br/>mpc_force_target = u_ref"]

        L13["Blend joint ref toward optimized pose"]
        L14["build_safe_targets()"]
        L15["compute_safe_tau()"]

        L16{"use_wbc?"}
        L17["Blend safe_tau & last_valid_support_tau<br/>based on support force & transition alpha"]
        L18["applied_tau = safe_tau"]

        L19["robot.set_joint_torques()<br/>robot.step()"]
        L20["Log states & metrics"]
    end

    subgraph Finally["Cleanup & Reporting"]
        F1["Restore original WBC gains"]
        F2["Compute RMSE / max slip / avg forces"]
        F3["Print MPC/WBC solve times"]
        F4["Plot CoM, contact force, torques"]
    end

    Init --> Loop
    Loop --> Loop
    Loop --> Finally
```

---

## Phase State Machine

```mermaid
stateDiagram-v2
    [*] --> INIT_SETTLE : start
    INIT_SETTLE --> DOUBLE_SUPPORT_HOLD : check_init_settle_transition()
    DOUBLE_SUPPORT_HOLD --> LOAD_SHIFT : check_double_support_transition()
    LOAD_SHIFT --> PRE_LIFTOFF : evaluate_load_shift_readiness()
    PRE_LIFTOFF --> SINGLE_SUPPORT : check_pre_liftoff_transition()
    SINGLE_SUPPORT --> [*] : end of simulation
```

---

## Single Iteration Detail

```mermaid
flowchart TB
    A["Start of step"]
    B["Estimator update<br/>(lock support in LOAD_SHIFT / PRE_LIFTOFF / SINGLE_SUPPORT)"]
    C["Update phase machine"]
    D["Phase transition triggered?"]
    E["Execute phase-entry hooks<br/>(gain override, contact re-init)"]
    F["Compute load shift metrics"]
    G["Filter support point"]
    H["Update single-support establishment"]
    I["Compute CoM target"]
    J["MPC set reference"]
    K{"Phase == SINGLE_SUPPORT?"}
    L["run_single_support_control()<br/>MPC solve -> WBC solve -> tau & f_ref"]
    M["Use nominal u_ref"]
    N["Blend joint angles toward optimized pose"]
    O["Build safe targets"]
    P["Compute safe torques"]
    Q{"use WBC?"}
    R["Blend WBC & safe torques<br/>based on support force / transition alpha"]
    S["Use safe torques directly"]
    T["Apply torques & step physics"]
    U["Log & print diagnostics"]
    V["End of step"]

    A --> B --> C --> D
    D -- Yes --> E --> F
    D -- No --> F
    F --> G --> H --> I --> J --> K
    K -- Yes --> L --> N
    K -- No --> M --> N
    N --> O --> P --> Q
    Q -- Yes --> R --> T
    Q -- No --> S --> T
    T --> U --> V
```

---

## `run_single_support_control()` Breakdown

```mermaid
flowchart LR
    A["MPC solve<br/>centroidal dynamics"]
    B["MPC force target<br/>f_ref"]
    C["Build WBC tasks:<br/>CoM, momentum, swing, support, posture"]
    D["Select WBC solver<br/>wbc_ss or wbc_ss_with_swing"]
    E["WBC QP solve<br/>tau + contact forces"]
    F["Return:<br/>f_ref, mpc_force_target<br/>wbc_result, active_solver, J_c, mpc_result"]

    A --> B --> C --> D --> E --> F
```

---

## Torque Blending Logic (SINGLE_SUPPORT)

```mermaid
flowchart TB
    A["safe_tau = compute_safe_tau()"]
    B["wbc_result available?"]
    C["Store wbc_result.tau as last_valid_support_tau"]
    D["Use cached last_valid_support_tau<br/>(or safe_tau if none)"]
    E["support_force = measured normal force"]
    F["transition_alpha = elapsed / TRANSITION_BLEND_TIME"]
    G["support_alpha = min(1, support_force / MIN_SUPPORT_FORCE)"]
    H["effective_alpha = min(MAX_TAU_BLEND,<br/>transition_alpha * support_alpha)"]
    I["tau_cmd[support] = (1-alpha)*safe_tau + alpha*last_valid_tau"]
    J["swing joints keep safe_tau"]

    A --> B
    B -- Yes --> C
    B -- No --> D
    C --> E --> F --> G --> H --> I --> J
    D --> E
```

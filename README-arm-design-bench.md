# 4-DOF Arm Design Bench

A single-file design and optimisation tool for a 4-DOF desktop robotic arm. It is not a
motion simulator — it sizes the machine. Open `arm-design-bench.html` in a browser; there
are no dependencies and no build step.

## What it does

- **Torque budget** for all four joints from a full moment summation, with servo
  utilisation, safety factor and the limiting joint called out.
- **Mass from geometry**: link mass is derived from the cross-section, wall thickness,
  infill percentage and material density — not entered by hand.
- **Structural checks**: root bending stress against a knocked-down yield strength, and
  tip deflection from an Euler–Bernoulli cantilever chain.
- **Centre of mass** and a rigid-body tipping check about the base support edge.
- **Optimiser** that searches link lengths to maximise reach (or payload) subject to every
  constraint, and reports which constraints are binding.
- **Parameter sweep** over thousands of link-length/payload combinations, with Pareto
  trade-off plots and a breakdown of what blocks the infeasible designs.
- **Reach envelope** showing the geometric workspace against the torque-feasible one.
- Every equation is written out, and every intermediate value is inspectable.

## Default configuration

| Joint | Servo | Stall torque |
|---|---|---|
| Base (yaw) | D2000ST | 21 kg·cm |
| Shoulder | MG995 | 11 kg·cm |
| Elbow | MG995 | 11 kg·cm |
| Wrist | MG995 | 11 kg·cm |

All editable, with a preset library (MG996R, DS3218, DS3225, MG90S, SG90) or custom values.

## The model

Internally everything is SI; conversion to mm / g / kg·cm happens only at the input and
display boundaries.

**Section properties.** A printed link is an outline of wall thickness *t* wrapped around a
core filled to *p*. Splitting the section at the wall line gives area and second moment
from the same decomposition, so mass and stiffness stay consistent:

```
A_eff = A_outline − (1 − p)·A_core
I_eff = I_outline − (1 − p)·I_core
m     = ρ · A_eff · L · (1 + k_hw)
```

**Static torque.** Gravity is vertical, so the moment arm about a horizontal pitch axis is
the *horizontal* offset:

```
τ_j,static = g · Σ_{i ∈ distal(j)} m_i (r_i − r_j)
```

**Inertial term.** `τ_j = |τ_j,static| + I_j·α`, with `I_j = Σ m_i·d_i²` about the joint
axis. Superposed as if both peak together, which is conservative.

**The base axis is different.** It rotates about a vertical axis, so gravity produces no
moment about it at all: `τ_base = I_base·α + τ_drag`. This is why an oversized base servo
still reads as nearly unloaded — a result the tool states explicitly rather than hiding.

**Capacity.** `τ_usable = τ_stall · k_derate`, `SF = τ_usable / τ_req ≥ SF_min`. Catalogue
stall torque is not continuously holdable; the derating fraction is the single most
influential input in the model.

**Deflection.** Each link is a cantilever under the transverse component of everything
outboard of it — end force, end moment, own distributed weight. Its end slope then rotates
the whole outboard chain, which is what makes the shoulder link dominate:

```
θ_i = (F_i L_i²/2 + M_i L_i + W_i L_i²/6) / EI_i
δ_i = (F_i L_i³/3 + M_i L_i²/2 + W_i L_i³/8) / EI_i
δ_tip = Σ_i [ n_i δ_i + θ_i × (p_TCP − p_i) ]
```

**Optimisation.** Maximise `R = L₁+L₂+L₃` subject to all joint safety factors, link
stresses, tip deflection, tipping and the link-length bounds, evaluated in the worst-case
pose (fully extended and horizontal). Solved by a bounded grid search followed by three
rounds of box refinement around the incumbent. Candidates are scored with the same solver
the analysis view uses, so the optimum is consistent with the rest of the tool by
construction.

## Assumptions worth knowing

- Point masses for servos, end effector and payload; link mass at the link midpoint.
- Quasi-static — no trajectory, no velocity-dependent friction, no back-EMF, no controller.
- Uniform prismatic sections; stress concentrations at bosses and bearing seats are not
  modelled, so reported bending stress is a nominal section stress.
- Infill treated as a homogeneous continuum. Good for mass; optimistic for stiffness at low
  percentages, where real gyroid or grid infill is anisotropic.
- Small deflections, computed on the undeformed geometry.
- No gearing, springs or counterbalance — a shoulder counterweight changes the picture
  entirely and is outside this model.

The tool states these in the Equations tab as well, next to the material data table.

## Verification

The solver is a pure function with no DOM dependency, which makes it directly testable. Its
outputs were checked against independent hand calculations for the default design
(section areas, link masses, joint moments, cantilever deflection) and against structural
invariants — for example, `∂τ_elbow/∂L₁` and `∂τ_wrist/∂L₂` come out as exact zeros in the
sensitivity table, since a proximal link length cannot load a distal joint.

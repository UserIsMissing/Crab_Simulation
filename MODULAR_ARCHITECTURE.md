# Modular Architecture Guide

## Overview

The simulator has been restructured into **4 independent, modular components**:

```
simulate_crab.py          Main loop (orchestrates everything)
├── config.py             All tunable parameters
├── sand_physics.py       RFT sand simulation (decoupled)
└── gait_controller.py    Leg control logic (decoupled)
```

This design makes it easy to **modify one aspect without breaking others**.

---

## Components

### 1. `config.py` - Central Configuration Hub

**Purpose**: One place to adjust all parameters

**What you can change here:**
- Sand stiffness and drag coefficients
- Which legs are active (currently only Right Middle)
- Gait timing and keyframes
- Simulation settings

**Example - Softer Sand:**
```python
# In config.py
SAND_CONFIG = {
    'K_VERTICAL': 150.0,    # Lower = softer/more sinkage
    'K_HORIZONTAL': 10.0,   # Lower = looser sand
    ...
}
```

**Example - Activate All Legs:**
```python
# In config.py, change:
LEG_TIPS = {
    'tip_rm': {..., 'active': True},  # Currently active
    'tip_rb': {..., 'active': False}, # Change to True
    'tip_rf': {..., 'active': False}, # Change to True
    ...
}
# Or programmatically:
config.activate_all_legs()
```

---

### 2. `sand_physics.py` - Granular Resistive Force (RFT)

**Purpose**: Encapsulates all sand physics logic

**Key Class:** `SandPhysics`
- Takes model, data, and optional sand parameters
- Computes vertical stiffness forces (sinkage)
- Computes horizontal drag forces (resistance)
- Applies forces only to active legs

**How it works:**
1. Checks if leg tip has penetrated ground (z < 0)
2. If yes: computes vertical force based on depth
3. Computes horizontal drag based on leg velocity
4. Applies both forces to the dactyl body

**Example - Change Sand Stiffness Mid-Simulation:**
```python
# In simulate_crab.py or elsewhere
sand_physics.set_sand_parameters(
    k_vertical=200.0,     # Harder sand
    k_horizontal=20.0     # More resistance
)
```

**Sand Physics Equations:**
- **Vertical Force**: $F_z = K_{vertical} \times depth$
- **Horizontal Drag**: $F_x = -K_{horizontal} \times v_x$, $F_y = -K_{horizontal} \times v_y$

Where:
- $K_{vertical}$ = stiffness (N/m) - controls how deep legs sink
- $K_{horizontal}$ = drag coefficient (N·s/m) - resists horizontal motion
- depth = how far tip penetrated below ground plane
- v = velocity of leg tip

---

### 3. `gait_controller.py` - Leg Motion Control

**Purpose**: Manages gait timing and joint angle targets

**Key Class:** `GaitController`
- Defines gait phases (Lift → Reach → Stab → Recover)
- Interpolates between keyframe positions
- Computes target angles for each joint

**Gait Structure:**
```
Phase 0 (Lift):   Leg lifts up
Phase 1 (Reach):  Leg extends forward
Phase 2 (Stab):   Leg pushes into sand
Phase 3 (Recover):Leg returns to start
[repeats]
```

**Keyframes** define joint angles at the end of each phase:
```python
GAIT_CONFIG = {
    'keyframes': [
        (0.0,  0.5),    # Initial (knee, ankle angles in radians)
        (-1.0, 0.1),    # End Lift phase
        (-1.0, -0.5),   # End Reach phase
        (1.0,  -0.5),   # End Stab phase
        (0.0,  0.5)     # End Recover phase → loops back
    ]
}
```

**Example - Faster Walking:**
```python
gait_controller.set_gait_parameters(cycle_duration=1.0)  # 1 sec instead of 2 sec
```

**Example - Different Keyframes (Different Gait):**
```python
new_keyframes = [
    (0.0,   0.3),    # Different start
    (-0.5,  0.2),    # Different lift
    (-0.8, -0.3),    # Different reach
    (0.8,  -0.4),    # Different stab
    (0.0,   0.3)     # Back to start
]
gait_controller.set_gait_parameters(keyframes=new_keyframes)
```

---

## Typical Workflows

### Experiment: Softer Sand

**Goal**: See if the crab can sink deeper into sand

**Changes:**
1. Edit `config.py`
2. Lower `K_VERTICAL` in `SAND_CONFIG`
3. Run simulation

```python
# config.py
SAND_CONFIG = {
    'K_VERTICAL': 100.0,    # Much softer sand
    'K_HORIZONTAL': 15.0,
    'PENETRATION_THRESHOLD': 0.0,
}
```

### Experiment: Different Gait Pattern

**Goal**: Test a new walking pattern

**Changes:**
1. Edit `config.py`
2. Modify `keyframes` in `GAIT_CONFIG`
3. Run simulation

```python
# config.py - Try a more aggressive stabbing motion
GAIT_CONFIG = {
    'cycle_duration': 2.0,
    'num_phases': 4,
    'keyframes': [
        (0.0,   0.8),    # Start higher
        (-1.5,  0.5),    # Lift higher
        (-1.5, -1.0),    # Reach further
        (1.5,  -1.0),    # Stab harder
        (0.0,   0.8)     # Return
    ]
}
```

### Experiment: Multiple Legs

**Goal**: Activate all 6 legs for hexapod walking

**Changes:**
1. Edit `config.py`
2. Set `'active': True` for all legs
3. Modify gait to use phase offsets for tripod gait

```python
# config.py
LEG_TIPS = {
    'tip_rm': {..., 'active': True},  # Right Middle
    'tip_rb': {..., 'active': True},  # Right Back
    'tip_rf': {..., 'active': True},  # Right Front
    'tip_lm': {..., 'active': True},  # Left Middle
    'tip_lb': {..., 'active': True},  # Left Back
    'tip_lf': {..., 'active': True},  # Left Front
}
```

Then in `simulate_crab.py`, add phase offsets to create staggered motions.

---

## How to Add New Features

### Add a New Leg

1. Define it in `crab.xml` (already done for all 6)
2. Add to `LEG_TIPS` in `config.py` with `'active': False`
3. Set `'active': True` to enable

### Change Joint Ranges

1. Edit `crab.xml` - modify `range` attribute on joints
2. Update `config.py` keyframes to stay within new ranges

### Add Logging/Analysis

1. Modify `sand_physics.py` to store force data
2. Add logging flag to `config.py`
3. Save forces to CSV for analysis

### Add Terrain Height Map

1. Create `terrain.py` module (similar to `sand_physics.py`)
2. Compute penetration depth from height map instead of z=0
3. Call `terrain.update()` in main loop

---

## File Dependency Graph

```
simulate_crab.py (main)
├─ imports ─ config.py (parameters)
├─ imports ─ gait_controller.py
│            └─ imports ─ config.py
└─ imports ─ sand_physics.py
             └─ imports ─ config.py
```

**Key Point:** `simulate_crab.py` doesn't know about sand physics equations or gait math. It just calls `gait_controller.update()` and `sand_physics.update()`. This separation makes the code **testable and maintainable**.

---

## Testing Individual Modules

### Test Sand Physics Independently
```python
from sand_physics import SandPhysics
import mujoco

model = mujoco.MjModel.from_xml_path('crab.xml')
data = mujoco.MjData(model)

sand = SandPhysics(model, data)
sand.print_status()
sand.set_sand_parameters(k_vertical=150.0)
```

### Test Gait Controller Independently
```python
from gait_controller import GaitController

gait = GaitController()
for t in range(0, 20, 1):  # 0-20 seconds
    targets = gait.compute_joint_targets(t)
    print(f"t={t}: knee={targets['knee_rm']:.2f}, ankle={targets['ankle_rm']:.2f}")
```

---

## Summary

| Module | Responsibility | Change When |
|--------|-----------------|-------------|
| `config.py` | Parameters | Tuning physics/gait, activating legs |
| `sand_physics.py` | Force calculations | Modifying sand model (not needed for tuning) |
| `gait_controller.py` | Motion timing | Creating new gaits (not needed for tuning) |
| `simulate_crab.py` | Orchestration | Adding new modules or data logging |
| `crab.xml` | Robot structure | Changing hardware dimensions |


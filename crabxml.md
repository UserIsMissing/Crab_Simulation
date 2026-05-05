# crab.xml Explanation

## Overview
`crab.xml` is a **MuJoCo MJCF (Multi-Joint dynamics with Contact) file** that defines the complete 3D model of the hexapod crab robot. It describes the robot's physical structure, materials, joints, actuators, and how different parts interact with each other and the environment.

---

## File Structure

### 1. **Compiler & Physics Settings**
```xml
<compiler angle="degree" coordinate="local" meshdir="meshes/"/>
<option gravity="0 0 -9.81" integrator="RK4" timestep="0.002"/>
```
- **`angle="degree"`**: Joint angles are specified in degrees (not radians)
- **`coordinate="local"`**: Use local coordinates for each body
- **`meshdir="meshes/"`**: Look for all 3D mesh files in the `meshes/` subdirectory
- **`gravity="0 0 -9.81"`**: Standard Earth gravity (9.81 m/s²) pointing downward
- **`integrator="RK4"`**: Use Runge-Kutta 4th-order integration for physics simulation
- **`timestep="0.002"`**: Simulation time step of 2 milliseconds

---

### 2. **Assets Section**
```xml
<asset>
    <material name="crab_r" rgba="0.8 0.2 0.2 1"/>          <!-- Red -->
    <material name="crab_l" rgba="0.2 0.8 0.8 1"/>          <!-- Cyan -->
    <material name="motor_grey" rgba="0.2 0.2 0.2 1"/>      <!-- Grey -->
    <material name="shell_white" rgba="0.9 0.9 0.9 1"/>     <!-- White -->
    
    <mesh name="shell_mesh" file="Shell.stl" scale="0.0254 0.0254 0.0254"/>
    <mesh name="nema17_mesh" file="Nema17.stl" scale="0.0254 0.0254 0.0254"/>
    <mesh name="femur_mesh" file="Femur.stl" scale="0.0254 0.0254 0.0254"/>
    <mesh name="merus_mesh" file="Merus.stl" scale="0.0254 0.0254 0.0254"/>
    <mesh name="dactyl_mesh" file="Dactyl.stl" scale="0.0254 0.0254 0.0254"/>
</asset>
```

**Materials**: Define colors for visualization (RGBA: Red, Green, Blue, Alpha)
- Right-side legs are shown in **red**
- Left-side legs are shown in **cyan**
- Motors are **grey**
- Shell is **white**

**Meshes**: Load 3D geometry from STL files
- **Scale "0.0254"**: Converts from inches (STL files are in inches) to meters (MuJoCo uses meters)
- Each segment of the leg (femur, merus, dactyl) is a separate mesh
- The motor (NEMA17) and shell (chassis) also have meshes

---

### 3. **World Body & Environment**
```xml
<worldbody>
    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
    <geom name="ground" type="plane" size="2 2 0.1" rgba=".9 .9 .9 0.5"/>
```
- **Light**: Positioned 3 meters above, pointing down for realistic lighting
- **Ground**: A flat plane (2m × 2m) where the robot can walk

---

### 4. **Base Shell (Robot Body)**
```xml
<body name="base_shell" pos="0 0 0.13">
    <freejoint/>
    <geom name="shell_visual" type="mesh" mesh="shell_mesh" mass="2.1"/>
```
- **Position**: Starts 0.13 m (13 cm) above ground
- **`<freejoint/>`**: The base can move freely in all 6 degrees of freedom (3 translation + 3 rotation)
- **Mass**: 2.1 kg (includes motors, wood base, and electronics)

---

### 5. **Leg Structure**

Each leg has the **same hierarchical structure**:

#### Right Middle Leg Example:
```xml
<body name="motor_rm" pos="0 -0.025 0.04">           <!-- Motor position on chassis -->
    <geom name="motor_rm_vis" type="mesh" mesh="nema17_mesh"/>
    
    <body name="femur_rm" pos="0.035 0.041 0.07">     <!-- Femur (upper leg) -->
        <joint name="hip_rm" type="hinge" axis="1 0 0" range="-90 90"/>
        <geom name="femur_rm_vis" type="mesh" mesh="femur_mesh"/>
        
        <body name="merus_rm" pos="0.04 -0.025 0.03">  <!-- Merus (middle leg) -->
            <joint name="knee_rm" type="hinge" axis="0 1 0" range="-90 90"/>
            <geom name="merus_rm_vis" type="mesh" mesh="merus_mesh"/>
            
            <body name="dactyl_rm" pos="0.04 0.025 0.0075">  <!-- Dactyl (toe/foot) -->
                <joint name="ankle_rm" type="hinge" axis="0 1 0" range="-60 60"/>
                <geom name="dactyl_rm_vis" type="mesh" mesh="dactyl_mesh"/>
                <site name="tip_rm" pos="0.048 -0.0125 -0.01"/>  <!-- Force sensor point -->
            </body>
        </body>
    </body>
</body>
```

**Naming Convention**:
- `r` = Right, `l` = Left
- `m` = Middle, `b` = Back, `f` = Front
- Example: `rm` = Right Middle, `lf` = Left Front

**Joints**:
- **Hip**: Rotates around X-axis (1 0 0), range ±90°
- **Knee**: Rotates around Y-axis (0 1 0), range ±90°
- **Ankle**: Rotates around Y-axis (0 1 0), range ±60°

**Sites**: Special marker points used to apply forces
- `tip_rm`, `tip_rb`, `tip_rf` = Right leg tips
- `tip_lm`, `tip_lb`, `tip_lf` = Left leg tips
- These are where the Granular Resistive Force (GRF) is applied in the simulation

**Total Legs**: 6 legs (3 per side)
- Right: Middle (rm), Back (rb), Front (rf)
- Left: Middle (lm), Back (lb), Front (lf)

---

### 6. **Actuators Section**
```xml
<actuator>
    <position name="hip_rm_srv" joint="hip_rm" kp="200"/>    <!-- Proportional gain -->
    <position name="knee_rm_srv" joint="knee_rm" kp="150"/>
    <position name="ankle_rm_srv" joint="ankle_rm" kp="150"/>
    <!-- ... (repeated for all 18 joints: 6 legs × 3 joints each) -->
</actuator>
```

**Position Control**: Each joint is controlled with a position target and gain (`kp`)
- **`kp="200"`** (Hip): Stiffer, harder to bend - represents NEMA 17 motor strength
- **`kp="150"`** (Knee/Ankle): Less stiff - represents MG996R servo characteristics
- Higher `kp` = faster/stronger response to position commands

**Total Actuators**: 18 (6 legs × 3 joints per leg)

---

### 7. **Dynamics Defaults**
```xml
<default>
    <joint damping="0.5" armature="0.01" stiffness="0"/>
</default>
```
- **`damping="0.5"`**: Friction/resistance in joints (simulates friction)
- **`armature="0.01"`**: Motor inertia (resistance to acceleration)
- **`stiffness="0"`**: No passive spring force in joints

---

### 8. **Contact Exclusions**
```xml
<contact>
    <exclude body1="motor_rm" body2="femur_rm"/>
    <exclude body1="femur_rm" body2="merus_rm"/>
    <exclude body1="merus_rm" body2="dactyl_rm"/>
    <!-- ... (repeated for all 6 legs) -->
</contact>
```

**Purpose**: Prevent collision detection between connected body segments
- Adjacent leg segments shouldn't collide with each other (they're connected by joints)
- This optimization reduces computational load

---

## Coordinate System

MuJoCo uses a **standard 3D coordinate system**:
- **X-axis**: Forward/Backward (positive = forward)
- **Y-axis**: Left/Right (positive = right)
- **Z-axis**: Up/Down (positive = up)

**Rotations** are specified as **Euler angles** (X, Y, Z rotations in degrees)

---

## How It All Works Together

1. **Physics Engine** reads gravity and timestep settings
2. **Base shell** (2.1 kg) is positioned and free to move
3. **6 legs** attach to the shell with **18 controllable joints**
4. **Actuators** receive commands to move each joint to target angles
5. **Dynamics** apply damping and inertia to make motion realistic
6. **Contact** system detects when dactyl tips touch the ground
7. **Sites** (tip positions) are used by `simulate_crab.py` to apply Granular Resistive Forces

---

## Key Parameters to Adjust

| Parameter | Location | Effect |
|-----------|----------|--------|
| `mass="2.1"` | Base shell | Total robot weight (affects gravity loading) |
| `kp="200"` | Hip actuators | Hip joint stiffness |
| `kp="150"` | Knee/ankle actuators | Leg stiffness |
| `damping="0.5"` | Default joints | Joint friction |
| `range="-90 90"` | Hip/knee joints | Maximum joint angles |
| `range="-60 60"` | Ankle joints | Maximum toe flexibility |


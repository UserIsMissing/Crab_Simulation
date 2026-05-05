# Crab-Inspired Sand Sifting Robot
**University of California Santa Cruz | Baskin School of Engineering**  
**Course:** ECE 216: Bio-Inspired Locomotion

## Overview
This repository contains a MuJoCo simulation environment for a hexapod robot designed to investigate brachyuran (crab-like) locomotion in granular media. The project specifically models "stabbing" and "pulling" gaits to maximize drawbar pull for sifting applications in **Upper Dry Beach Sand**.

---

## Hardware-Informed Physics
The simulation is grounded in real-world hardware specifications to ensure accurate sinkage and torque modeling.

### Mass Calculation Breakdown
*   **Actuators**: 4x NEMA 17 (1.40 kg) + 8x MG996R Servos (0.44 kg).
*   **Chassis**: 0.5" Wood Oval base (0.11 kg).
*   **Limbs**: 12x 3D-Printed segments (0.14 kg).
*   **Total System Mass**: **2.09 kg**.

### Soil Mechanics: Upper Dry Beach Sand
We utilize **Granular Resistive Force Theory (RFT)** to override standard contact physics in `simulate_crab.py`.
*   **Vertical Stiffness ($K_{vertical}$)**: 250.0 N/m.
*   **Horizontal Drag ($K_{horizontal}$)**: 15.0 N·s/m.
*   **Equilibrium Sinkage**: At 2.09 kg, the dactyl tips penetrate approximately **2.7 cm** before reaching static equilibrium.

---

## Installation

1.  **Clone the Repository**:
    ```bash
    git clone https://github.com/[your-username]/[repo-name].git
    cd [repo-name]
    ```

2.  **Install Dependencies**:
    Ensure you have Python 3.8+ and the following packages installed:
    ```bash
    pip install mujoco numpy matplotlib
    ```

---

## Running the Simulation

### 1. Configure the Robot (`crab.xml`)
The robot's physical properties are defined in `crab.xml`. Ensure the `base_shell` mass is set to match the hardware calculations:
```xml
<body name="base_shell" pos="0 0 0.13">
    <geom name="shell_visual" type="mesh" mesh="shell_mesh" mass="2.09"/>
</body>
```

### 2. Execute the Environment (`simulate_crab.py`)
Run the main simulation loop. This script handles the gait generation and the RFT force injection.
```bash
python simulate_crab.py
```

**Key Simulation Controls:**
*   **Gait Period**: Defined by `cycle_duration` (default: 4.0s) in `simulate_crab.py`.
*   **Force Application**: The `apply_rft_forces()` function monitors the dactyl tip sites (`tip_rm`, `tip_l`, etc.) and applies forces only when the global vertical position $z < 0$.

---

## Project Structure
*   `crab.xml`: MuJoCo MJCF file defining robot morphology, mass, and actuators.
*   `simulate_crab.py`: Python script for the simulation loop and RFT physics.
*   `meshes/`: STL/OBJ files for the 3D-printed leg segments and wood chassis.
*   `data/`: Directory for saved force and displacement graphs for preliminary results.

---

## Academic Context
This work is part of an inquiry into how biological movements like crab gaits can be applied to granular terrain navigation and sifting. The model accounts for effective weight (W), vertical resistive force (Fz), and horizontal drag/thrust (Fh).
```
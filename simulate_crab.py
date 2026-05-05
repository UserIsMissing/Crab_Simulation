"""
Main Simulation Loop - Crab Locomotion in Sand
Orchestrates gait control and sand physics
"""

import mujoco
import mujoco.viewer
import time

# Import modular components
from gait_controller import GaitController
from sand_physics import SandPhysics
import config

# ============================================================================
# INITIALIZATION
# ============================================================================

# Load the MuJoCo model and initialize data
model = mujoco.MjModel.from_xml_path('crab.xml')
data = mujoco.MjData(model)

# Initialize modular components
gait_controller = GaitController()
sand_physics = SandPhysics(model, data)

# Print configurations
if config.SIMULATION_CONFIG['verbose']:
    print("\n" + "="*50)
    print("CRAB LOCOMOTION SIMULATOR - Initialization")
    print("="*50)
    sand_physics.print_status()
    gait_controller.print_status(0.0)

# ============================================================================
# SIMULATION LOOP
# ============================================================================

with mujoco.viewer.launch_passive(model, data) as viewer:
    print("▶ Beginning Brachyuran-inspired walking cycle...\n")
    
    while viewer.is_running():
        step_start = time.time()
        
        # Get current simulation time
        sim_time = data.time
        
        # 1. UPDATE GAIT - compute target joint angles
        joint_targets = gait_controller.compute_joint_targets(sim_time)
        
        # 2. APPLY CONTROLS - set joint targets
        # Map targets to control indices based on XML actuator order
        # Actuator order in crab.xml: hip_rm, knee_rm, ankle_rm, hip_rb, ..., ankle_lf
        control_mapping = {
            'hip_rm': 0,   'knee_rm': 1,   'ankle_rm': 2,
            'hip_rb': 3,   'knee_rb': 4,   'ankle_rb': 5,
            'hip_rf': 6,   'knee_rf': 7,   'ankle_rf': 8,
            'hip_lm': 9,   'knee_lm': 10,  'ankle_lm': 11,
            'hip_lb': 12,  'knee_lb': 13,  'ankle_lb': 14,
            'hip_lf': 15,  'knee_lf': 16,  'ankle_lf': 17,
        }
        
        for joint_name, target_value in joint_targets.items():
            ctrl_idx = control_mapping.get(joint_name)
            if ctrl_idx is not None:
                data.ctrl[ctrl_idx] = target_value
        
        # 3. UPDATE SAND PHYSICS - apply RFT forces
        sand_physics.update()
        
        # 4. STEP SIMULATION
        mujoco.mj_step(model, data)
        viewer.sync()
        
        # 5. REAL-TIME SYNCHRONIZATION
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
        
        # 6. DEBUG OUTPUT (optional)
        if config.SIMULATION_CONFIG['verbose'] and int(sim_time * 10) % 10 == 0:
            # Print status every 1 second
            pass  # Uncomment below to debug
            # gait_controller.print_status(sim_time)
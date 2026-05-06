"""
Main Simulation Loop - Crab Locomotion in Sand
Orchestrates gait control and sand physics
"""

import mujoco
import mujoco.viewer
import time
import threading

# Try to import keyboard library for pause/play functionality
try:
    import keyboard
    KEYBOARD_AVAILABLE = True
except ImportError:
    KEYBOARD_AVAILABLE = False
    print("Warning: 'keyboard' module not found. Pause/play disabled.")
    print("Install with: pip install keyboard\n")

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

# Set visual ground position to match penetration threshold from config
ground_visual_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'ground_visual')
if ground_visual_id >= 0:
    model.geom_pos[ground_visual_id, 2] = config.SAND_CONFIG['PENETRATION_THRESHOLD']

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

paused = False  # Pause/play state

def toggle_pause():
    """Callback for spacebar to toggle pause/play"""
    global paused
    paused = not paused
    status = "PAUSED" if paused else "RUNNING"
    print(f"Simulation {status} at t={data.time:.2f}s")

# Register keyboard listener if available
if KEYBOARD_AVAILABLE:
    keyboard.add_hotkey('space', toggle_pause)

with mujoco.viewer.launch_passive(model, data) as viewer:
    if KEYBOARD_AVAILABLE:
        print("▶ Beginning Brachyuran-inspired walking cycle...")
        print("   Press SPACE to pause/resume")
        print("   Close window to exit\n")
    else:
        print("▶ Beginning Brachyuran-inspired walking cycle...\n")
    
    while viewer.is_running():
        step_start = time.time()
        
        if not paused:
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
        
        # 5. RENDER (always render, even when paused)
        viewer.sync()
        
        # 6. REAL-TIME SYNCHRONIZATION (skip sleep if paused)
        if not paused:
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
        else:
            # When paused, just render at a reasonable frame rate
            time.sleep(0.016)  # ~60 FPS
        
        # 7. DEBUG OUTPUT (optional)
        if config.SIMULATION_CONFIG['verbose'] and int(data.time * 10) % 10 == 0:
            # Print status every 1 second
            pass  # Uncomment below to debug
            # gait_controller.print_status(data.time)
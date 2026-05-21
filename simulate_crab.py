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

# Actuator order mirrors crab.xml — built once, used every step
CONTROL_MAPPING = {
    'hip_rm': 0,  'knee_rm': 1,  'ankle_rm': 2,
    'hip_rb': 3,  'knee_rb': 4,  'ankle_rb': 5,
    'hip_rf': 6,  'knee_rf': 7,  'ankle_rf': 8,
    'hip_lm': 9,  'knee_lm': 10, 'ankle_lm': 11,
    'hip_lb': 12, 'knee_lb': 13, 'ankle_lb': 14,
    'hip_lf': 15, 'knee_lf': 16, 'ankle_lf': 17,
}

# Speed control — - slows down, = speeds up
SPEED_STEPS = [0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0]
speed_idx = SPEED_STEPS.index(1.0)
speed_multiplier = 1.0
paused = True  # Start paused to allow initial observation of the setup

# Phase tracking — print on phase change and every PHASE_PRINT_INTERVAL sim-seconds
PHASE_PRINT_INTERVAL = 0.5   # lower = more frequent updates, higher = less spam
_prev_phase_A = -1
_prev_phase_B = -1
_last_print_t  = -PHASE_PRINT_INTERVAL  # force a print on the very first step


def key_callback(keycode):
    """MuJoCo viewer key handler (GLFW key codes).

    MuJoCo's built-in +/- speed control has no effect in launch_passive mode
    because our Python loop owns the timing.  We intercept the same keycodes
    here so the +/- buttons shown in the viewer UI actually work.
    """
    global paused, speed_multiplier, speed_idx

    if keycode == 32:                       # SPACE — pause / resume
        paused = not paused
        state = "PAUSED" if paused else "RUNNING"
        print(f"[{state}]  t = {data.time:.2f}s")

    elif keycode in (61, 334):              # = / numpad+  →  faster
        speed_idx = min(speed_idx + 1, len(SPEED_STEPS) - 1)
        speed_multiplier = SPEED_STEPS[speed_idx]
        print(f"Speed: {speed_multiplier}x real-time")

    elif keycode in (45, 333):              # - / numpad-  →  slower
        speed_idx = max(speed_idx - 1, 0)
        speed_multiplier = SPEED_STEPS[speed_idx]
        print(f"Speed: {speed_multiplier}x real-time")


with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    print("▶  Brachyuran-inspired walking cycle starting...")
    print("   SPACE       pause / resume")
    print("   -  /  =     slow down / speed up  (0.1x – 8x real-time)")
    print("   Close window to exit\n")

    while viewer.is_running():
        step_start = time.time()

        if not paused:
            sim_time = data.time

            # 1. Compute target joint angles for all active legs
            joint_targets = gait_controller.compute_joint_targets(sim_time)

            # 2. Push targets to actuators
            for joint_name, target_value in joint_targets.items():
                ctrl_idx = CONTROL_MAPPING.get(joint_name)
                if ctrl_idx is not None:
                    data.ctrl[ctrl_idx] = target_value

            # 3. Apply RFT sand forces
            sand_physics.update()

            # 4. Advance physics
            mujoco.mj_step(model, data)

            # 5. Phase tracking — print on phase change or every PHASE_PRINT_INTERVAL
            off_1 = config.LEG_PHASE_OFFSETS['tip_lf']
            off_2 = config.LEG_PHASE_OFFSETS['tip_rf']
            pi_1  = gait_controller.get_phase_info(sim_time + off_1 * gait_controller.cycle_duration)
            pi_2  = gait_controller.get_phase_info(sim_time + off_2 * gait_controller.cycle_duration)

            phase_changed = (pi_1['phase_index'] != _prev_phase_A or
                             pi_2['phase_index'] != _prev_phase_B)
            time_due      = (sim_time - _last_print_t) >= PHASE_PRINT_INTERVAL

            if phase_changed or time_due:
                _prev_phase_A = pi_1['phase_index']
                _prev_phase_B = pi_2['phase_index']
                _last_print_t = sim_time
                print(
                    f"t={sim_time:6.2f}s │ "
                    f"G1 [lf,lb,rm↕]: {pi_1['phase_name']:<8} {pi_1['time_in_phase']*100:3.0f}% │ "
                    f"G2 [rf,rb,lm↕]: {pi_2['phase_name']:<8} {pi_2['time_in_phase']*100:3.0f}%"
                )

        # 5. Render every iteration (including while paused)
        viewer.sync()

        # 6. Real-time pacing — divide timestep by speed multiplier so that
        #    >1x sleeps less (faster) and <1x sleeps more (slow-motion).
        if not paused:
            target_step = model.opt.timestep / speed_multiplier
            elapsed = time.time() - step_start
            remaining = target_step - elapsed
            if remaining > 0:
                time.sleep(remaining)
        else:
            time.sleep(0.016)   # ~60 FPS while paused
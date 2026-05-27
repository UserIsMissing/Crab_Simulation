"""
Main Simulation Loop - Crab Locomotion in Sand with Data Logging
Tracks: Horizontal Drag, Slippage Velocity, and Mechanical Energy Exerted
"""
import mujoco
import mujoco.viewer
import time
import numpy as np

# Import modular components
from gait_controller import GaitController
from sand_physics import SandPhysics
import config

# Initialize MuJoCo model and data
model = mujoco.MjModel.from_xml_path('crab.xml')
data = mujoco.MjData(model)

# Match visual ground position to configuration threshold
ground_visual_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'ground_visual')
if ground_visual_id >= 0:
    model.geom_pos[ground_visual_id, 2] = config.SAND_CONFIG['PENETRATION_THRESHOLD']

# Initialize modular components
gait_controller = GaitController()
sand_physics = SandPhysics(model, data)

CONTROL_MAPPING = {
    'hip_rm': 0,  'knee_rm': 1,  'ankle_rm': 2,
    'hip_rb': 3,  'knee_rb': 4,  'ankle_rb': 5,
    'hip_rf': 6,  'knee_rf': 7,  'ankle_rf': 8,
    'hip_lm': 9,  'knee_lm': 10, 'ankle_lm': 11,
    'hip_lb': 12, 'knee_lb': 13, 'ankle_lb': 14,
    'hip_lf': 15, 'knee_lf': 16, 'ankle_lf': 17,
}

# Data collection container tailored for terramechanics and energy analytics
data_history = {
    'time': [],
    'pos_x': [],
    'slippage_velocity': [],
    'total_drag_force_x': [],
    'total_vertical_force_z': [],
    'cumulative_energy_joules': []
}

# Variable tracking for energy integration
cumulative_energy = 0.0
last_time = 0.0

print("\n" + "="*50)
print("RUNNING CRAB SAND INTERACTION & ENERGY DATA LOGGER")
print("="*50)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()
        sim_time = data.time
        dt = sim_time - last_time if sim_time > 0 else model.opt.timestep
        
        # 1. Compute asymmetric gait controls from active controller
        targets = gait_controller.compute_joint_targets(sim_time)
        
        # 2. Assign target values directly to MuJoCo actuators
        for name, val in targets.items():
            if name in CONTROL_MAPPING:
                data.ctrl[CONTROL_MAPPING[name]] = val

        # 3. Process RFT sand interaction updates
        sand_physics.update()
        
        # Extract applied reaction forces from the sand physics directly
        force_z_accum = 0.0
        force_x_accum = 0.0
        for tip in config.get_active_legs():
            body_name = config.get_leg_body(tip)
            if body_name:
                body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
                if body_id >= 0:
                    # xfrc_applied contains forces acting directly on the dactyl segments
                    force_x_accum += abs(data.xfrc_applied[body_id, 0])
                    force_z_accum += abs(data.xfrc_applied[body_id, 2])
                    
        # 4. Calculate Slippage Velocity (Lateral body velocity ignoring intentional forward target)
        # qvel[0] is translation velocity along the X axis
        chassis_vel_x = data.qvel[0]
        
        # 5. Calculate Energy Exerted (Power = Torque * Joint Velocity)
        # We integrate power over time across all 18 joints to compute total Joules
        instantaneous_power = 0.0
        for i in range(model.nu):
            torque = data.actuator_force[i]
            joint_vel = data.qvel[model.jnt_dofadr[model.actuator_trnid[i, 0]]]
            instantaneous_power += abs(torque * joint_vel)
            
        cumulative_energy += instantaneous_power * dt

        # 6. Append measurements to log history
        data_history['time'].append(sim_time)
        data_history['pos_x'].append(data.qpos[0]) 
        data_history['slippage_velocity'].append(abs(chassis_vel_x))
        data_history['total_drag_force_x'].append(force_x_accum)
        data_history['total_vertical_force_z'].append(force_z_accum)
        data_history['cumulative_energy_joules'].append(cumulative_energy)

        # 7. Physics step
        mujoco.mj_step(model, data)
        viewer.sync()

        # Update timing reference
        last_time = sim_time

        # Real-time synchronization pacing
        elapsed = time.time() - step_start
        if elapsed < model.opt.timestep:
            time.sleep(model.opt.timestep - elapsed)
            
        # Break simulation window automatically after 12 seconds of clear data cycles
        if sim_time >= 60.0:
            print("Target simulation data window captured successfully.")
            break


# Export dataset safely to working folder using config fallback filename
log_target_file = config.SIMULATION_CONFIG.get('log_file', 'simulation_log.npy')
if log_target_file.endswith('.csv'):
    log_target_file = log_target_file.replace('.csv', '.npy')

np.save(log_target_file, data_history)
print(f"Simulation logged data saved to '{log_target_file}'")
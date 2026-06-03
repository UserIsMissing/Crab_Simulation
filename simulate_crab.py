"""
Main Simulation Loop — Crab Locomotion in Sand with Data Logging
Tracks: Horizontal Drag, Slippage Velocity, Mechanical Energy Exerted,
        and Rake-Sifting Parasitic Drag Penalties.
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
    'hip_lf': 15, 'knee_lf': 16, 'ankle_lf': 17
}
# Pre-fetch Sifter Body ID for high-efficiency reference
sifter_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'sifter_assembly')
if sifter_body_id < 0:
    print("[WARNING] 'sifter_assembly' body not detected in crab.xml. Disabling sifter force modeling.")

# Structure dictionary arrays for simulation history logging
data_history = {
    'time': [],
    'pos_x': [],
    'slippage_velocity': [],
    'total_drag_force_x': [],
    'total_vertical_force_z': [],
    'sifter_drag_force_x': [],
    'sifter_vertical_force_z': [],
    'cumulative_energy_joules': []
}

# Runtime initialization properties
cumulative_energy = 0.0
last_time = 0.0

print("\nStarting full hexapod simulation loop...")
print("Press ESC in the viewer window to close.")

# Launch active visualization window environment
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

        # 1. Fetch current simulation runtime state clock
        sim_time = data.time
        dt = sim_time - last_time if last_time > 0.0 else model.opt.timestep

        # 2. Query and apply parametric tripod gait positions natively
        target_positions = gait_controller.compute_joint_targets(sim_time)        
        for joint_name, position_val in target_positions.items():
            if joint_name in CONTROL_MAPPING:
                actuator_idx = CONTROL_MAPPING[joint_name]
                data.ctrl[actuator_idx] = position_val

        # 3. Apply Resistive Force Theory (RFT) via native sand_physics.update()
        # This automatically resets external forces and computes active foot interactions
        data.xfrc_applied.fill(0.0)
        sand_physics.update()

        # 4. Calculate accumulated foot forces from xfrc_applied for logging metrics
        force_x_accum = 0.0
        force_z_accum = 0.0
        for tip_name in config.get_active_legs():
            if tip_name in sand_physics._body_ids:
                b_id = sand_physics._body_ids[tip_name]
                force_x_accum += data.xfrc_applied[b_id, 0]
                force_z_accum += data.xfrc_applied[b_id, 2]

        # 5. Calculate terramechanic load penalties experienced by the sifting rake
        sifter_fx = 0.0
        sifter_fz = 0.0
        
        if sifter_body_id >= 0:
            # Explicitly match your XML naming strings exactly
            tine_prefixes = ['tine_0'] + [f'tine_l{i}' for i in range(1, 10)] + [f'tine_r{i}' for i in range(1, 10)]
            
            k_vert = config.SAND_CONFIG.get('K_VERTICAL', 180.0)
            drag_coeff = config.SIFTER_CONFIG.get('RAKE_DRAG_COEFF', 45.0)
            tine_dia = config.SIFTER_CONFIG.get('tine_diameter_m', 0.004)
            sand_surface = config.SAND_CONFIG.get('PENETRATION_THRESHOLD', 0.0)
            
            # Fetch global positional matrix data of the sifter tool frame
            sifter_xpos = data.xpos[sifter_body_id]
            sifter_cvel = data.cvel[sifter_body_id]
            linear_vx = sifter_cvel[3]  # Global translational X velocity
            
            found_any_tines = False
            for tine_name in tine_prefixes:
                geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, tine_name)
                
                if geom_id >= 0:
                    found_any_tines = True
                    # Get the local Z center position of the cylinder
                    local_z_center = model.geom_pos[geom_id, 2]
                    
                    # MuJoCo cylinder geom size array is [radius, half_length]
                    tine_half_length = model.geom_size[geom_id, 1]
                    
                    # Calculate the true global Z position of the absolute bottom tip of the cylinder
                    tine_bottom_tip_z = sifter_xpos[2] + local_z_center - tine_half_length
                    
                    # If the bottom tip is below the sand surface, interaction forces are active!
                    if tine_bottom_tip_z < sand_surface:
                        # Depth calculation is how far the tip has penetrated into the sand plane
                        tine_depth = sand_surface - tine_bottom_tip_z
                        
                        # Cap depth calculation to the maximum length of the tine
                        tine_depth = min(tine_depth, tine_half_length * 2.0)
                        
                        # Apply RFT equations natively per individual tine component
                        tine_fz = k_vert * 0.5 * tine_depth
                        tine_fx = -drag_coeff * (tine_dia * tine_depth) * linear_vx
                        
                        sifter_fx += tine_fx
                        sifter_fz += tine_fz
            
            # Apply cumulative calculated forces back into the MuJoCo system vectors
            data.xfrc_applied[sifter_body_id, 0] = sifter_fx
            data.xfrc_applied[sifter_body_id, 2] = sifter_fz

        # 6. Compute structural chassis state metrics & instantaneous mechanical power
        chassis_vel_x = data.qvel[0] # Root chassis translational velocity
        
        instantaneous_power = 0.0
        for i in range(model.nu):
            torque = data.actuator_force[i]
            joint_vel = data.qvel[model.jnt_dofadr[model.actuator_trnid[i, 0]]]
            instantaneous_power += abs(torque * joint_vel)
            
        cumulative_energy += instantaneous_power * dt

        # 7. Append active spatial measurements to runtime history logging dict
        data_history['time'].append(sim_time)
        data_history['pos_x'].append(data.qpos[0]) 
        data_history['slippage_velocity'].append(abs(chassis_vel_x))
        data_history['total_drag_force_x'].append(abs(force_x_accum))
        data_history['total_vertical_force_z'].append(force_z_accum)
        data_history['sifter_drag_force_x'].append(abs(sifter_fx))
        data_history['sifter_vertical_force_z'].append(sifter_fz)
        data_history['cumulative_energy_joules'].append(cumulative_energy)

        # 8. Step physics model forward
        mujoco.mj_step(model, data)
        viewer.sync()

        # Update tracking step clocks
        last_time = sim_time

        # Real-time synchronization pacing loop constraints
        elapsed = time.time() - step_start
        if elapsed < model.opt.timestep:
            time.sleep(model.opt.timestep - elapsed)
            
        # Target test collection timeout window
        if sim_time >= 60.0:
            print("Target 60-second simulation data window captured successfully.")
            break

# Export completed telemetry logs out to binary file layout
log_target_file = config.SIMULATION_CONFIG.get('log_file', 'simulation_log.npy')
if log_target_file.endswith('.csv'):
    log_target_file = log_target_file.replace('.csv', '.npy')

np.save(log_target_file, data_history)
print(f"Data telemetry exported safely to '{log_target_file}'.")
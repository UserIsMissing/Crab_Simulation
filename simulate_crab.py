import mujoco
import mujoco.viewer
import time
import numpy as np

# 1. Load the model and data
model = mujoco.MjModel.from_xml_path('crab.xml')
data = mujoco.MjData(model)

# RFT Coefficients (Adjust these to change sand "thickness")
# Low-Density / Loose Sand (The "Desert" Model)
# K_VERTICAL = 200–300   # Vertical stiffness (N/m)

# Loose/Dry Sand
# K_HORIZONTAL = 10.0 – 20.0   # Horizontal drag (N·s/m)

K_VERTICAL = 250.0    # Vertical stiffness (N/m)
K_HORIZONTAL = 15.0   # Horizontal drag (N·s/m), beach standard

LEG_MAPPING = {
    'tip_rm': 'dactyl_rm',
    'tip_rb': 'dactyl_rb',
    'tip_rf': 'dactyl_rf',
    'tip_l':  'dactyl_l'
}

def apply_rft_forces(model, data):
    """
    Iterates through all defined legs and applies RFT forces 
    to each dactyl body based on its specific tip penetration.
    """
    for site_name, body_name in LEG_MAPPING.items():
        try:
            tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            
            # Get the global position of this specific tip
            tip_pos = data.site_xpos[tip_id]
            
            # Check if this specific dactyl has entered the sand (z < 0)
            if tip_pos[2] < 0:
                depth = abs(tip_pos[2])
                
                # 1. Vertical Force (Stiffness)
                f_z = K_VERTICAL * depth
                
                # 2. Horizontal Forces (Drag/Thrust)
                tip_vel = np.zeros(6) 
                mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_SITE, tip_id, tip_vel, 0)
                
                f_x = -K_HORIZONTAL * tip_vel[3]
                f_y = -K_HORIZONTAL * tip_vel[4]
                
                # Apply 3D force to this specific body
                data.xfrc_applied[body_id] = [f_x, f_y, f_z, 0, 0, 0]
            else:
                # Reset forces for this leg when it's in the air
                data.xfrc_applied[body_id] = [0, 0, 0, 0, 0, 0]
        except ValueError:
            # Skip if a name in the mapping isn't found in the XML
            continue

def get_lerp(start, end, percentage):
    """Linearly interpolates between start and end values."""
    return start + (end - start) * percentage

def update_gait(model, data):
    # --- MODULAR TIMING CONFIG ---
    cycle_duration = 2.0  
    num_phases = 4
    step = cycle_duration / num_phases  # x seconds per phase
    
    t = data.time % cycle_duration
    phase_index = int(t // step)
    p_time = (t % step) / step  # Normalized time (0.0 to 1.0) within current phase

    # --- POSITION CONFIG (Knee, Ankle) ---
    # Define the keyframes for the end of each phase
    # Phase 0: Lift, Phase 1: Reach, Phase 2: Stab, Phase 3: Recover
    keyframes = [
        (0.0,  0.5),   # Initial/Recovery State
        (-1.0, 0.1),   # End of Phase 0 (Lift)
        (-1.0, -0.5),  # End of Phase 1 (Reach)
        (1.0,  -0.5),  # End of Phase 2 (Stab)
        (0.0,  0.5)    # End of Phase 3 (Recover/Loop)
    ]

    # Get start and end keyframes for the current phase
    start_knee, start_ankle = keyframes[phase_index]
    end_knee, end_ankle = keyframes[phase_index + 1]

    # --- APPLY CONTROLS ---
    # Set Hips to 0
    for i in [0, 3, 6, 9]:
        data.ctrl[i] = 0

    # Smoothly transition Knee and Ankle for the Right Middle Leg
    data.ctrl[1] = get_lerp(start_knee, end_knee, p_time)
    data.ctrl[2] = get_lerp(start_ankle, end_ankle, p_time)

    # Apply the same logic to other legs 
    # or add a 'phase_offset' to create a tripod gait.
        
# 2. Launch the viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Beginning Brachyuran-inspired walking cycle...")
    
    while viewer.is_running():
        step_start = time.time()

        # Update controls and sand physics
        update_gait(model, data)
        apply_rft_forces(model, data)
        
        mujoco.mj_step(model, data)
        viewer.sync()

        # Real-time synchronization
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
"""
Sand Physics Module - Granular Resistive Force Theory (RFT)
Handles force calculations based on leg penetration and velocity in sand.
Completely decoupled from gait control.
"""

import numpy as np
import mujoco
import config


class SandPhysics:
    """Encapsulates RFT-based sand physics simulation"""
    
    def __init__(self, model, data, sand_params=None):
        """
        Initialize sand physics engine
        
        Args:
            model: MuJoCo model
            data: MuJoCo data
            sand_params: dict with 'K_VERTICAL', 'K_HORIZONTAL', 'PENETRATION_THRESHOLD'
                        Uses config defaults if None
        """
        self.model = model
        self.data = data
        
        # Use provided parameters or fall back to config
        if sand_params is None:
            sand_params = config.SAND_CONFIG
        
        self.K_VERTICAL = sand_params.get('K_VERTICAL', 250.0)
        self.K_HORIZONTAL = sand_params.get('K_HORIZONTAL', 15.0)
        self.PENETRATION_THRESHOLD = sand_params.get('PENETRATION_THRESHOLD', 0.0)
        
        # Cache for tip/body IDs (computed once, reused)
        self._tip_ids = {}
        self._body_ids = {}
        self._initialize_ids()
    
    def _initialize_ids(self):
        """Pre-compute and cache MuJoCo object IDs for efficiency"""
        for tip_name in config.LEG_TIPS.keys():
            try:
                tip_id = mujoco.mj_name2id(
                    self.model, 
                    mujoco.mjtObj.mjOBJ_SITE, 
                    tip_name
                )
                body_name = config.LEG_TIPS[tip_name]['body']
                body_id = mujoco.mj_name2id(
                    self.model, 
                    mujoco.mjtObj.mjOBJ_BODY, 
                    body_name
                )
                
                self._tip_ids[tip_name] = tip_id
                self._body_ids[tip_name] = body_id
                
            except ValueError:
                # Skip if not found in XML
                pass
    
    def update(self):
        """
        Apply RFT forces to all active legs.
        Call this once per simulation step.
        """
        for tip_name in config.get_active_legs():
            if tip_name not in self._tip_ids:
                continue
            
            tip_id = self._tip_ids[tip_name]
            body_id = self._body_ids[tip_name]
            
            # Get tip position and check penetration
            tip_pos = self.data.site_xpos[tip_id]
            penetration_depth = self.PENETRATION_THRESHOLD - tip_pos[2]
            
            if penetration_depth > 0:
                # Leg is in the sand - apply forces
                forces = self._compute_rft_forces(tip_id, penetration_depth)
                self.data.xfrc_applied[body_id] = forces
            else:
                # Leg is in the air - no forces
                self.data.xfrc_applied[body_id] = [0, 0, 0, 0, 0, 0]
    
    def _compute_rft_forces(self, tip_id, depth):
        """
        Compute RFT forces for a given leg tip
        
        Args:
            tip_id: MuJoCo site ID of the leg tip
            depth: How deep the tip has penetrated into the sand
            
        Returns:
            [f_x, f_y, f_z, tau_x, tau_y, tau_z] force/torque vector
        """
        # 1. VERTICAL FORCE (Stiffness-based resistance)
        # F_z = K_vertical * depth
        f_z = self.K_VERTICAL * depth
        
        # 2. HORIZONTAL FORCES (Velocity-dependent drag)
        # Get the velocity of the tip in global frame
        tip_vel = np.zeros(6)  # [linear_vel, angular_vel]
        mujoco.mj_objectVelocity(
            self.model, 
            self.data, 
            mujoco.mjtObj.mjOBJ_SITE, 
            tip_id, 
            tip_vel, 
            0
        )
        
        # Horizontal velocity components (x and y)
        v_x = tip_vel[3]
        v_y = tip_vel[4]
        
        # Apply linear drag (proportional to velocity, opposite direction)
        f_x = -self.K_HORIZONTAL * v_x
        f_y = -self.K_HORIZONTAL * v_y
        
        # No torques (only force application)
        return [f_x, f_y, f_z, 0, 0, 0]
    
    def set_sand_parameters(self, k_vertical=None, k_horizontal=None, threshold=None):
        """
        Update sand physics parameters on-the-fly
        
        Args:
            k_vertical: New vertical stiffness (N/m), or None to keep current
            k_horizontal: New horizontal drag (N·s/m), or None to keep current
            threshold: New penetration threshold (m), or None to keep current
        """
        if k_vertical is not None:
            self.K_VERTICAL = k_vertical
        if k_horizontal is not None:
            self.K_HORIZONTAL = k_horizontal
        if threshold is not None:
            self.PENETRATION_THRESHOLD = threshold
    
    def get_sand_parameters(self):
        """Return current sand parameters as dict"""
        return {
            'K_VERTICAL': self.K_VERTICAL,
            'K_HORIZONTAL': self.K_HORIZONTAL,
            'PENETRATION_THRESHOLD': self.PENETRATION_THRESHOLD,
        }
    
    def print_status(self):
        """Print current sand physics configuration"""
        print("\n=== Sand Physics Configuration ===")
        print(f"Vertical Stiffness: {self.K_VERTICAL} N/m")
        print(f"Horizontal Drag: {self.K_HORIZONTAL} N·s/m")
        print(f"Penetration Threshold: {self.PENETRATION_THRESHOLD} m")
        print(f"Active Legs: {config.get_active_legs()}")
        print("===================================\n")

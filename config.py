"""
Configuration module for Crab Simulator
Centralized control over physics parameters, leg mappings, and gait settings
"""

# ============================================================================
# SAND/GROUND PHYSICS - Granular Resistive Force Theory (RFT)
# ============================================================================
# These values simulate Upper Dry Beach Sand behavior
# Reference: README.md mass calculation = 2.09 kg → ~2.7 cm sinkage at equilibrium

SAND_CONFIG = {
    # Vertical stiffness (N/m) - controls how rigid the sand feels
    # Higher = harder sand (less sinkage), Lower = softer sand (more sinkage)
    'K_VERTICAL': 180.0,
    
    # Horizontal drag coefficient (N·s/m) - controls lateral resistance
    # Higher = more resistance to horizontal motion (sticky sand)
    # Lower = less resistance (loose sand)
    'K_HORIZONTAL': 25.0,
    
    # Penetration threshold (m) - depth at which RFT forces start applying
    # Typically 0 (forces apply when z < 0, i.e., below ground plane)
    'PENETRATION_THRESHOLD': 0.01,
}

# ============================================================================
# LEG CONTROL MAPPING
# ============================================================================
# Maps dactyl tips (sensors) to dactyl bodies (where forces are applied)
# Extended format: supports all 6 legs for future use
# Currently controlled: only 'rm' (Right Middle)

LEG_TIPS = {
    # Right Side Legs
    'tip_rm': {'body': 'dactyl_rm', 'name': 'Right Middle', 'active': True},
    'tip_rb': {'body': 'dactyl_rb', 'name': 'Right Back', 'active': False},
    'tip_rf': {'body': 'dactyl_rf', 'name': 'Right Front', 'active': False},
    
    # Left Side Legs
    'tip_lm': {'body': 'dactyl_lm', 'name': 'Left Middle', 'active': False},
    'tip_lb': {'body': 'dactyl_lb', 'name': 'Left Back', 'active': False},
    'tip_lf': {'body': 'dactyl_lf', 'name': 'Left Front', 'active': False},
}

# ============================================================================
# GAIT TIMING & CONTROL
# ============================================================================

GAIT_CONFIG = {
    # Total cycle duration (seconds) - time for one complete gait cycle
    'cycle_duration': 2.0,
    
    # Number of phases per cycle
    # Phase 0: Lift, Phase 1: Reach, Phase 2: Stab, Phase 3: Recover
    'num_phases': 4,
    
    # Hip control (mostly for balance, currently set to neutral)
    'hip_target': 0.0,  # Radians or normalized units
    
    # Keyframe positions for gait animation (knee, ankle angles)
    # Each tuple is (knee_angle, ankle_angle) at the END of each phase
    # Phases: [Init, Lift, Reach, Stab, Recover/Loop]
    'keyframes': [
        (0.0,  0.5),   # Initial/Recovery State
        (-1.0, 0.1),   # End of Phase 0 (Lift leg up)
        (-1.0, -0.5),  # End of Phase 1 (Reach forward)
        (1.0,  -0.5),  # End of Phase 2 (Stab into sand)
        (0.0,  0.5)    # End of Phase 3 (Recover to start)
    ],
}

# ============================================================================
# SIMULATION PARAMETERS
# ============================================================================

SIMULATION_CONFIG = {
    # Render the visualization
    'use_viewer': True,
    
    # Print debug information during simulation
    'verbose': True,
    
    # Log forces and states for analysis
    'log_forces': False,
    'log_file': 'simulation_log.csv',
}

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def get_active_legs():
    """Returns list of active leg tip names"""
    return [tip_name for tip_name, info in LEG_TIPS.items() if info['active']]

def get_leg_body(tip_name):
    """Returns the body name for a given tip sensor"""
    if tip_name in LEG_TIPS:
        return LEG_TIPS[tip_name]['body']
    return None

def activate_leg(leg_name):
    """Enable control for a specific leg"""
    if leg_name in LEG_TIPS:
        LEG_TIPS[leg_name]['active'] = True

def deactivate_leg(leg_name):
    """Disable control for a specific leg"""
    if leg_name in LEG_TIPS:
        LEG_TIPS[leg_name]['active'] = False

def activate_all_legs():
    """Enable all 6 legs (for multi-leg control in future)"""
    for leg in LEG_TIPS:
        LEG_TIPS[leg]['active'] = True

def deactivate_all_legs():
    """Disable all legs"""
    for leg in LEG_TIPS:
        LEG_TIPS[leg]['active'] = False

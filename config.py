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
    # reverse=True  → leg runs its keyframe sequence backwards in time.
    #
    # For rightward locomotion:
    #   rf, rb  — normal:   reach right, plant, pull body right
    #   rm      — reversed: contracts while rf/rb extend (inverted from right side)
    #   lf, lb  — reversed: push body right (mirror of right-side motion)
    #   lm      — normal:   contracts while lf/lb push (inverted-of-reversed = normal)
    'tip_rf': {'body': 'dactyl_rf', 'name': 'Right Front',  'active': True,  'reverse': False},
    'tip_rb': {'body': 'dactyl_rb', 'name': 'Right Back',   'active': True,  'reverse': False},
    'tip_rm': {'body': 'dactyl_rm', 'name': 'Right Middle', 'active': True,  'reverse': True},
    'tip_lf': {'body': 'dactyl_lf', 'name': 'Left Front',   'active': True,  'reverse': True},
    'tip_lb': {'body': 'dactyl_lb', 'name': 'Left Back',    'active': True,  'reverse': True},
    'tip_lm': {'body': 'dactyl_lm', 'name': 'Left Middle',  'active': True,  'reverse': False},
}

# ============================================================================
# GAIT PHASE OFFSETS
# ============================================================================
# Right side and left side alternate 180° out of phase.
# reverse=True (in LEG_TIPS) makes a leg run its keyframes backwards —
# no extra phase offset needed for the middle legs.

LEG_PHASE_OFFSETS = {
    'tip_rf': 0.0,   # Right group
    'tip_rb': 0.0,   # Right group
    'tip_rm': 0.0,   # Right group  (reversed → contracts while rf/rb extend)
    'tip_lf': 0.5,   # Left group
    'tip_lb': 0.5,   # Left group
    'tip_lm': 0.5,   # Left group   (normal  → contracts while lf/lb push)
}

# ============================================================================
# GAIT TIMING & CONTROL
# ============================================================================

GAIT_CONFIG = {
    # Total cycle duration (seconds) - time for one complete gait cycle
    'cycle_duration': 2.0,

    # Number of phases per cycle
    # Phase 0: Lift, Phase 1: Reach, Phase 2: Stab, Phase 3: Recover
    'num_phases': 5,

    # Hip control (mostly for balance, currently set to neutral)
    'hip_target': 0.0,  # Radians or normalized units

    # Seconds to blend from the neutral standing pose into the full gait.
    # Prevents the violent torque spike at t=0 that flips the robot.
    'warmup_duration': 2.0,

    # Keyframe positions for gait animation (knee, ankle angles in radians)
    # Each tuple is (knee_angle, ankle_angle) at the END of each phase.
    # Phases: [Init, Lift, Reach, Stab, Recover/Loop]
    # Note: keyframes[0] also serves as the neutral standing pose for warmup.
    'keyframes': [
        # (knee, ankle) in radians
        # knee negative = leg lifts up   |   knee positive = leg pushes down
        # ankle negative = toe extends   |   ankle positive = toe curls back

        # Testing keyframes for visual debugging
        # (-0.2, -0.2),
        # (-0.2, -0.2),
        # (-0.2, -0.2),
        # (-0.2, -0.2),
        # (-0.2, -0.2)
        
        (-0.2, -0.2),   # [0] Standing pose / recovery end
        (-0.8, -0.2),     # [1] Lift    — knee folds all the way up
        (-0.8, -0.8),     # [2] stretch — ankle extends up
        (0.0, -0.8),     # [3] Reach   — knee lowers to touch ground
        (0.8, 0.8),   # [4] Stab    — knee pushes down into sand (reduced from 1.0)
        (-0.2,  -0.2)   # [5] Recover — return to standing pose
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

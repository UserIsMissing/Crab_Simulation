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
    # mirror=True     → right-side leg: all angles negated (geometry correction).
    # keyframe_set    → which sequence this leg uses.
    #   'standard'    → front and back legs
    #   'middle'      → middle legs (tune separately to fix geometry differences)
    'tip_lf': {'body': 'dactyl_lf', 'name': 'Left Front',   'active': True, 'mirror': False, 'keyframe_set': 'standard'},
    'tip_lb': {'body': 'dactyl_lb', 'name': 'Left Back',    'active': True, 'mirror': False, 'keyframe_set': 'standard'},
    'tip_lm': {'body': 'dactyl_lm', 'name': 'Left Middle',  'active': True, 'mirror': False, 'keyframe_set': 'middle'},
    'tip_rf': {'body': 'dactyl_rf', 'name': 'Right Front',  'active': True, 'mirror': True,  'keyframe_set': 'standard'},
    'tip_rb': {'body': 'dactyl_rb', 'name': 'Right Back',   'active': True, 'mirror': True,  'keyframe_set': 'standard'},
    'tip_rm': {'body': 'dactyl_rm', 'name': 'Right Middle', 'active': True, 'mirror': True,  'keyframe_set': 'middle'},
}

# ============================================================================
# GAIT PHASE OFFSETS
# ============================================================================
# Group A (0.0): lf, lb, rm — lift and step together
# Group B (0.5): rf, rb, lm — hold the robot stable while A moves, then step
#
# The 0.5 offset is the ONLY thing making the groups oppose each other.
# Group B runs the same keyframe sequence, starting half a cycle later —
# when A is lifting, B is mid-push; when A plants, B starts lifting.
# No extra negation is applied for being in Group B.

LEG_PHASE_OFFSETS = {
    'tip_lf': 0.0,   # Group A
    'tip_lb': 0.0,   # Group A
    'tip_rm': 0.0,   # Group A  (mirror=True handles geometry, not group)
    'tip_rf': 0.5,   # Group B  (mirror=True handles geometry, not group)
    'tip_rb': 0.5,   # Group B  (mirror=True handles geometry, not group)
    'tip_lm': 0.5,   # Group B
}

# ============================================================================
# GAIT TIMING & CONTROL
# ============================================================================

GAIT_CONFIG = {
    # Total cycle duration (seconds) - time for one complete gait cycle
    'cycle_duration': 4.0,

    # Number of phases per cycle — must equal len(keyframe_sets[*]) - 1
    'num_phases': 5,

    # Hip control (mostly for balance, currently set to neutral)
    'hip_target': 0.0,  # Radians or normalized units

    # Seconds to blend from the neutral standing pose into the full gait.
    # Prevents the violent torque spike at t=0 that flips the robot.
    'warmup_duration': 2.0,

    # Two named keyframe sets — (knee, ankle) tuples, one per phase boundary.
    # knee negative = leg lifts up   |  knee positive = leg pushes down
    # ankle negative = toe extends   |  ankle positive = toe curls back
    # Right-side legs get all values negated automatically via mirror=True.
    'keyframe_sets': {

        # ── Front and back legs (lf, lb, rf, rb) ──────────────────────────
        'standard': [
            (-0.8, -0.8),   # [0]  standing / recovery
            ( 0.4, -0.8),   # [2]
            ( 0.8, 0.6),   # [3]
            (-0.2,  0.8),   # [4]
            (-0.4,  0.2),   # [5]
            (-0.8, -0.8),   # [10] back to standing

            # (-0.8, -0.8),   # [0]  standing / recovery
            # ( 0.2, -0.8),   # [1]
            # ( 0.4, -0.8),   # [2]
            # ( 0.6, -0.6),   # [3]
            # ( 0.8,  0.2),   # [4]
            # ( 0.8,  0.5),   # [5]
            # ( 0.2,  0.8),   # [6]
            # (-0.2,  1.0),   # [7]
            # (-0.6,  0.8),   # [8]
            # (-0.8, -0.2),   # [9]
            # (-0.8, -0.8),   # [10] back to standing
        ],

        # ── Middle legs (lm, rm) — tune these to match middle-leg geometry ─
        'middle': [
            (-0.8, -0.8),   # [0]  standing / recovery
            ( 0.4, -0.8),   # [2]
            ( 0.8, 0.6),   # [3]
            (-0.2,  0.8),   # [4]
            (-0.4,  0.2),   # [5]
            (-0.8, -0.8),   # [10] back to standing

            # (-0.8, -0.8),   # [0]  standing / recovery
            # ( 0.2, -0.8),   # [1]
            # ( 0.4, -0.8),   # [2]
            # ( 0.6, -0.6),   # [3]
            # ( 0.8,  0.2),   # [4]
            # ( 0.8,  0.5),   # [5]
            # ( 0.2,  0.8),   # [6]
            # (-0.2,  1.0),   # [7]
            # (-0.6,  0.8),   # [8]
            # (-0.8, -0.2),   # [9]
            # (-0.8, -0.8),   # [10] back to standing
        ],
    },
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

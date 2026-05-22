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
    # keyframe_set → which angle sequence this leg follows.
    # 'right' / 'left' keyframes are written with explicit angles per side —
    # no automatic mirroring.  Middle legs use the same set as their side's
    # front/back but are offset 0.5 in LEG_PHASE_OFFSETS.
    'tip_rf': {'body': 'dactyl_rf', 'name': 'Right Front',  'active': True, 'keyframe_set': 'right'},
    'tip_rb': {'body': 'dactyl_rb', 'name': 'Right Back',   'active': True, 'keyframe_set': 'right'},
    'tip_rm': {'body': 'dactyl_rm', 'name': 'Right Middle', 'active': True, 'keyframe_set': 'right'},
    'tip_lf': {'body': 'dactyl_lf', 'name': 'Left Front',   'active': True, 'keyframe_set': 'left'},
    'tip_lb': {'body': 'dactyl_lb', 'name': 'Left Back',    'active': True, 'keyframe_set': 'left'},
    'tip_lm': {'body': 'dactyl_lm', 'name': 'Left Middle',  'active': True, 'keyframe_set': 'left'},
}

# ============================================================================
# GAIT PHASE OFFSETS
# ============================================================================
# Front/back legs on each side move in sync (offset 0.0).
# Middle legs use the same keyframe set as their side but start half a cycle
# later (offset 0.5) — automatically matching the opposing side's timing.
LEG_PHASE_OFFSETS = {
    'tip_rf': 0.0,   # Right front  — right cycle start
    'tip_rb': 0.0,   # Right back   — right cycle start
    'tip_rm': 0.5,   # Right middle — 0.5 into right cycle (auto-syncs with left side)
    'tip_lf': 0.0,   # Left front   — left cycle start
    'tip_lb': 0.0,   # Left back    — left cycle start
    'tip_lm': 0.5,   # Left middle  — 0.5 into left cycle (auto-syncs with right side)
}

# ============================================================================
# GAIT TIMING & CONTROL
# ============================================================================

GAIT_CONFIG = {
    # Total cycle duration (seconds) - time for one complete gait cycle
    'cycle_duration': 4.0,

    # MUST be even — so the 0.5 phase offset between groups lands exactly on kf[N/2],
    # which is neutral (0,0).  Both groups are neutral simultaneously at the handoff.
    'num_phases': 6,

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

        # ── Right side (rf, rb, rm) ───────────────────────────────────────
        # Write the actual joint angles you want for right-side legs.
        # kf[0] = kf[3] = kf[6] = (0,0) keeps the 50%-mark at neutral so
        # rm (0.5 offset) is always neutral when rf/rb are neutral.
        'right': [
            ( 0.0,  0.0),   # [0] neutral
            ( 0.8,  0.2),   # [1] knee down
            ( 0.6,  0.4),   # [2] full tuck
            (-0.6,  0.8),   # [3] knee back up
            (-0.4, -0.6),   # [4] full extension
            ( 0.2, -0.4),   # [5] knee back down
            ( 0.0,  0.0),   # [8] neutral — loops to [0]
        ],

        # ── Left side (lf, lb, lm) ───────────────────────────────────────
        # Write the actual joint angles you want for left-side legs.
        # lm (0.5 offset) will automatically be at kf[3] when lf/lb are at kf[0].
        'left': [
            ( 0.0,  0.0),   # [0] neutral
            (-0.4, -0.4),   # [1] knee down
            (-0.8,  0.4),   # [2] full tuck
            ( 0.8,  0.8),   # [3] knee back up
            (-0.8,  0.4),   # [4] full extension
            ( 0.2,  0.2),   # [5] knee back down
            ( 0.0,  0.0),   # [8] neutral — loops to [0]
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

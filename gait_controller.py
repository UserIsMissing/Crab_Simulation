"""
Gait Controller Module - Handles leg motion control
Manages timing, keyframe interpolation, and joint control.
Completely independent from sand physics.
"""

import numpy as np
import config


class GaitController:
    """Encapsulates gait timing and leg motion control"""
    
    def __init__(self, gait_params=None):
        """
        Initialize gait controller
        
        Args:
            gait_params: dict with 'cycle_duration', 'num_phases', 'keyframes'
                        Uses config defaults if None
        """
        if gait_params is None:
            gait_params = config.GAIT_CONFIG
        
        self.cycle_duration   = gait_params.get('cycle_duration', 2.0)
        self.num_phases       = gait_params.get('num_phases', 4)
        self.keyframes        = gait_params.get('keyframes', [])
        self.hip_target       = gait_params.get('hip_target', 0.0)
        self.warmup_duration  = gait_params.get('warmup_duration', 2.0)

        # Validate keyframes
        if len(self.keyframes) != self.num_phases + 1:
            raise ValueError(
                f"Number of keyframes ({len(self.keyframes)}) must equal "
                f"num_phases + 1 ({self.num_phases + 1})"
            )

        # Compute phase duration
        self.phase_duration = self.cycle_duration / self.num_phases
    
    def get_phase_info(self, sim_time):
        """
        Get current phase and interpolation progress
        
        Args:
            sim_time: Current simulation time (seconds)
            
        Returns:
            {
                'phase_index': int (0 to num_phases-1),
                'time_in_phase': float (0.0 to 1.0),
                'cycle_progress': float (0.0 to 1.0),
                'current_time': float,
                'phase_name': str
            }
        """
        # Wrap time to current cycle
        wrapped_time = sim_time % self.cycle_duration
        
        # Calculate phase
        phase_index = int(wrapped_time // self.phase_duration)
        phase_index = min(phase_index, self.num_phases - 1)
        
        # Normalized time within this phase (0.0 to 1.0)
        time_in_phase = (wrapped_time % self.phase_duration) / self.phase_duration
        
        # Overall cycle progress
        cycle_progress = wrapped_time / self.cycle_duration
        
        # Phase names for debugging
        phase_names = ['Lift', 'Reach', 'Stab', 'Recover']
        phase_name = phase_names[phase_index] if phase_index < len(phase_names) else 'Unknown'
        
        return {
            'phase_index': phase_index,
            'time_in_phase': time_in_phase,
            'cycle_progress': cycle_progress,
            'current_time': wrapped_time,
            'phase_name': phase_name,
        }
    
    @staticmethod
    def _smooth_step(t):
        """Cosine ease-in/ease-out — velocity is zero at both endpoints.
        Eliminates the abrupt velocity jump at every phase boundary."""
        return (1.0 - np.cos(np.pi * t)) / 2.0

    def interpolate_keyframes(self, phase_index, progress):
        """
        Interpolate between keyframes using cosine easing for smooth motion.

        Args:
            phase_index: Current phase (0 to num_phases-1)
            progress: Normalized progress through phase (0.0 to 1.0)

        Returns:
            (knee_angle, ankle_angle) interpolated values
        """
        start_knee, start_ankle = self.keyframes[phase_index]
        end_knee, end_ankle = self.keyframes[phase_index + 1]

        t = self._smooth_step(progress)
        knee  = start_knee  + (end_knee  - start_knee)  * t
        ankle = start_ankle + (end_ankle - start_ankle) * t

        return knee, ankle
    
    def compute_joint_targets(self, sim_time):
        """
        Compute target angles for all joints at current time.
        Each active leg uses its own phase offset from config.LEG_PHASE_OFFSETS
        to produce the tripod alternating pattern.

        Args:
            sim_time: Current simulation time (seconds)

        Returns:
            {
                'hip_rm': float, 'knee_rm': float, 'ankle_rm': float,
                'hip_rb': float, 'knee_rb': float, 'ankle_rb': float,
                ... (for all active legs)
            }
        """
        # Warmup: blend from keyframes[0] (neutral standing pose) to the full
        # gait target over warmup_duration seconds.  Prevents the large torque
        # spike at t=0 that causes the robot to bounce/flip on startup.
        neutral_knee, neutral_ankle = self.keyframes[0]
        warmup_blend = self._smooth_step(min(sim_time / self.warmup_duration, 1.0))

        targets = {}

        for leg_name, leg_info in config.LEG_TIPS.items():
            if not leg_info['active']:
                continue

            leg_id = leg_name.replace('tip_', '')

            # Shift this leg's clock by its phase offset so tripod groups
            # are 180° out of phase with each other.
            phase_offset = config.LEG_PHASE_OFFSETS.get(leg_name, 0.0)
            leg_time = sim_time + phase_offset * self.cycle_duration

            phase_info = self.get_phase_info(leg_time)

            if leg_info.get('reverse', False):
                # Run keyframes backwards: at cycle_progress p, use (1-p) % 1.
                # When group mates are in Lift (p≈0, going up),
                # this leg runs backward through Recover/Stab (going down).
                rev_p       = (1.0 - phase_info['cycle_progress']) % 1.0
                rev_wrapped = rev_p * self.cycle_duration
                rev_idx     = min(int(rev_wrapped // self.phase_duration), self.num_phases - 1)
                rev_t       = (rev_wrapped % self.phase_duration) / self.phase_duration
                gait_knee, gait_ankle = self.interpolate_keyframes(rev_idx, rev_t)
            else:
                gait_knee, gait_ankle = self.interpolate_keyframes(
                    phase_info['phase_index'], phase_info['time_in_phase']
                )

            # During warmup, interpolate from neutral toward the gait target
            knee  = neutral_knee  + (gait_knee  - neutral_knee)  * warmup_blend
            ankle = neutral_ankle + (gait_ankle - neutral_ankle) * warmup_blend

            targets[f'hip_{leg_id}']   = self.hip_target
            targets[f'knee_{leg_id}']  = knee
            targets[f'ankle_{leg_id}'] = ankle

        return targets
    
    def set_gait_parameters(self, cycle_duration=None, num_phases=None, keyframes=None):
        """
        Update gait parameters on-the-fly
        
        Args:
            cycle_duration: New cycle duration (seconds), or None to keep current
            num_phases: New number of phases, or None to keep current
            keyframes: New keyframes list, or None to keep current
        """
        if cycle_duration is not None:
            self.cycle_duration = cycle_duration
            self.phase_duration = self.cycle_duration / self.num_phases
        
        if num_phases is not None:
            self.num_phases = num_phases
            self.phase_duration = self.cycle_duration / self.num_phases
        
        if keyframes is not None:
            if len(keyframes) != self.num_phases + 1:
                raise ValueError(
                    f"Number of keyframes ({len(keyframes)}) must equal "
                    f"num_phases + 1 ({self.num_phases + 1})"
                )
            self.keyframes = keyframes
    
    def print_status(self, sim_time):
        """Print current gait state for all active legs"""
        print(f"\n=== Gait Status (t={sim_time:.2f}s) ===")
        print(f"Cycle duration: {self.cycle_duration}s  |  Phase duration: {self.phase_duration:.2f}s")

        for leg_name, leg_info in config.LEG_TIPS.items():
            if not leg_info['active']:
                continue
            phase_offset = config.LEG_PHASE_OFFSETS.get(leg_name, 0.0)
            leg_time = sim_time + phase_offset * self.cycle_duration
            pi = self.get_phase_info(leg_time)
            print(f"  {leg_info['name']:14s} (offset={phase_offset:.1f}) → "
                  f"{pi['phase_name']:7s} {pi['time_in_phase']*100:5.1f}%")

        print("=====================================\n")

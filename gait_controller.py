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
        
        self.cycle_duration = gait_params.get('cycle_duration', 2.0)
        self.num_phases = gait_params.get('num_phases', 4)
        self.keyframes = gait_params.get('keyframes', [])
        self.hip_target = gait_params.get('hip_target', 0.0)
        
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
    
    def interpolate_keyframes(self, phase_index, progress):
        """
        Interpolate between keyframes for smooth motion
        
        Args:
            phase_index: Current phase (0 to num_phases-1)
            progress: Normalized progress through phase (0.0 to 1.0)
            
        Returns:
            (knee_angle, ankle_angle) interpolated values
        """
        start_knee, start_ankle = self.keyframes[phase_index]
        end_knee, end_ankle = self.keyframes[phase_index + 1]
        
        # Linear interpolation
        knee = start_knee + (end_knee - start_knee) * progress
        ankle = start_ankle + (end_ankle - start_ankle) * progress
        
        return knee, ankle
    
    def compute_joint_targets(self, sim_time):
        """
        Compute target angles for all joints at current time
        
        Args:
            sim_time: Current simulation time (seconds)
            
        Returns:
            {
                'hip_rm': float, 'knee_rm': float, 'ankle_rm': float,
                'hip_rb': float, 'knee_rb': float, 'ankle_rb': float,
                ... (for all 6 legs)
            }
        """
        phase_info = self.get_phase_info(sim_time)
        phase_idx = phase_info['phase_index']
        progress = phase_info['time_in_phase']
        
        # Get interpolated keyframe values
        knee, ankle = self.interpolate_keyframes(phase_idx, progress)
        
        targets = {}
        
        # Currently only Right Middle leg is active
        # Hip is kept neutral, Knee and Ankle follow gait
        for leg_name in config.LEG_TIPS.keys():
            if config.LEG_TIPS[leg_name]['active']:
                # Extract leg identifier (e.g., 'rm' from 'tip_rm')
                leg_id = leg_name.replace('tip_', '')
                
                targets[f'hip_{leg_id}'] = self.hip_target
                targets[f'knee_{leg_id}'] = knee
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
        """Print current gait state"""
        phase_info = self.get_phase_info(sim_time)
        targets = self.compute_joint_targets(sim_time)
        
        print(f"\n=== Gait Status (t={sim_time:.2f}s) ===")
        print(f"Phase: {phase_info['phase_name']} (#{phase_info['phase_index']})")
        print(f"Progress: {phase_info['time_in_phase']*100:.1f}% through phase")
        print(f"Cycle: {phase_info['cycle_progress']*100:.1f}% complete")
        print(f"\nJoint Targets:")
        for joint_name, angle in targets.items():
            print(f"  {joint_name}: {angle:.2f} rad")
        print("=====================================\n")

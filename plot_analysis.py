"""
plot_analysis.py — Processes sand terramechanics and energy variables.
Saves figure as sand_interaction_analysis.png

run ./mujoco_env/bin/mjpython simulate_crab.py

then run python(3) plot_analysis.py
"""
import numpy as np
import matplotlib.pyplot as plt
import os
import config

# Determine log source file from configuration string matching the simulation script
data_file = config.SIMULATION_CONFIG.get('log_file', 'simulation_log.npy')
if data_file.endswith('.csv'):
    data_file = data_file.replace('.csv', '.npy')

if not os.path.exists(data_file):
    raise FileNotFoundError(f"Missing logged dataset '{data_file}'. Please run 'simulate_crab.py' first.")

# Load binary dict structure
data_history = np.load(data_file, allow_pickle=True).item()
time_vec = data_history['time']

# Clean academic plotting layout overrides
plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
fig, axs = plt.subplots(2, 1, figsize=(10, 12), sharex=True)

fig.suptitle('Analysis of Dynamic Drag and Media Slippage', 
             fontsize=14, fontweight='bold', y=0.97)

# ------------------------------------------------------------------
# PANEL 1: Sand Interaction Forces (Drag vs Support)
# ------------------------------------------------------------------
axs[0].plot(time_vec, data_history['total_drag_force_x'], label='Total Horizontal Sand Drag Force ($F_x$)', color='#e74c3c', linewidth=2)
axs[0].plot(time_vec, data_history['total_vertical_force_z'], label='Total Vertical Ground Reaction ($F_z$)', color='#2980b9', linewidth=1.5, alpha=0.7)
axs[0].set_ylabel('Force [Newtons]', fontsize=10, fontweight='bold')
axs[0].set_title('A. Granular Resistive Force Theory (RFT) Loading Profiles', fontsize=11, loc='left', color='#2c3e50', fontweight='bold')
axs[0].legend(loc='upper right', frameon=True, facecolor='white', framealpha=0.9)

# ------------------------------------------------------------------
# PANEL 2: Media Slippage vs Forward Translation
# ------------------------------------------------------------------
# Calculate an index of slip progress (Slippage vs actual X progress)
axs[1].plot(time_vec, data_history['slippage_velocity'], label='Chassis Slippage Velocity ($v_x$)', color='#e67e22', linewidth=2)
axs[1].plot(time_vec, data_history['pos_x'], label='Net Displacement Position (X)', color='#2ecc71', linewidth=2, linestyle='--')
axs[1].set_ylabel('Velocity [m/s] / Position [m]', fontsize=10, fontweight='bold')
axs[1].set_title('B. Dynamic Locomotion Slippage & Progress Verification', fontsize=11, loc='left', color='#2c3e50', fontweight='bold')
axs[1].legend(loc='upper left', frameon=True, facecolor='white', framealpha=0.9)
axs[1].set_xlabel('Time [seconds]', fontsize=10, fontweight='bold')

# Finalize layout constraints and save file
plt.tight_layout(rect=[0, 0, 1, 0.95])
output_img = 'sand_interaction_analysis.png'
plt.savefig(output_img, dpi=300, bbox_inches='tight')

print(f"\nTerramechanics dashboard compiled! Figure saved successfully as '{output_img}' (300 DPI PNG).")
plt.show()
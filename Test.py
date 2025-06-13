# -----------------------------
# Description: Simulate and visualize an autonomous drone docking algorithm 
# in a 3D space, with obstacle avoidance and cost-based path planning
# -----------------------------

import numpy as np
import matplotlib.pyplot as plt
from path_planning import compute_optimal_docking_path

# -----------------------------
# Example Scenario Setup
# -----------------------------

# Fly at 5 m/s and start in different parts of the space
posR = np.array([10, 10, 5]) # Receiver drone
posS = np.array([30, 30, 5]) # Supplier drone
vR = 5 
vS = 5

# 3D No-Fly Zones (NFZs) w/ spherical obstacles
NFZ_centers = np.array([ 
    [20, 20, 5],
    [15, 15, 8],
    [25, 10, 6]
])
NFZ_radii = np.array([4, 5, 6])

# 
bounds = {
    'x': [0, 40],
    'y': [0, 40],
    'z': [0, 20]
}

# -----------------------------
# Objective: 
# - Search for a valid docking point
# - Plan obstacle-free trajectories from posR and posS to the docking point
# - Evaluate and return the cost (based on distance or energy)
# -----------------------------

# Call the docking path planner (IMPORTANT)
docking_point, pathR, pathS, docking_cost = compute_optimal_docking_path(
    posR, posS, vR, vS,
    NFZ_centers, NFZ_radii,
    bounds,
    cost_type='energy', alpha=1.0, beta=10.0, v=5
)

# -----------------------------
# Visualization via matplotlib
# -----------------------------

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_title(f'Docking Point Found. Cost = {docking_cost:.3f}')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.grid(True)

ax.scatter(*posR, color='blue', s=100, label='Receiver')
ax.scatter(*posS, color='green', s=100, label='Supplier')
ax.scatter(*docking_point, color='black', s=150, label='Docking Point')
ax.plot(pathR[:,0], pathR[:,1], pathR[:,2], 'b--', linewidth=2, label='Path R')
ax.plot(pathS[:,0], pathS[:,1], pathS[:,2], 'g--', linewidth=2, label='Path S')

phi, theta = np.meshgrid(np.linspace(0, np.pi, 10), np.linspace(0, 2 * np.pi, 20))
for center, radius in zip(NFZ_centers, NFZ_radii):
    xs = radius * np.sin(phi) * np.cos(theta) + center[0]
    ys = radius * np.sin(phi) * np.sin(theta) + center[1]
    zs = radius * np.cos(phi) + center[2]
    ax.plot_surface(xs, ys, zs, alpha=0.3, color='red', edgecolor='none')

ax.legend()
ax.set_xlim(bounds['x'])
ax.set_ylim(bounds['y'])
ax.set_zlim(bounds['z'])
plt.show()

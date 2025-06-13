"""
Objective: Take a smoothed RRT* path between the receiver and supplier, and then:
1). Evaluate every point along that path as a possible docking point
2). Assign a cost to each point based on: Total path effort (J_path), Altitude preference (J_alt)
and Symmetry/skew between both drones (J_skew)
3). Pick the best docking point — lowest combined cost
"""

import numpy as np
from run_rrt_star import run_rrt_star_3d_plus
from smooth_path import smooth_rrt_path

def get_docking_rrt_star(posR, posS, NFZ_centers, NFZ_radii, bounds, cost_type, alpha, beta):
    v = 5  # constant velocity for time-based

    # Run RRT*+ and returns a single path between the drones (not separate ones yet) by finding a viable path in 3D space while avoiding all NFZs.
    path_smooth, cost, success = run_rrt_star_3d_plus(posR, posS, NFZ_centers, NFZ_radii, bounds, cost_type, alpha, beta)

    if not success:
        return None, float('inf'), None, None

    # Path evaluation weights
    z_pref = posR[2]
    w_path = 20.0
    w_alt = 1.0
    w_skew = 1.0

    N_init = path_smooth.shape[0]
    Ndim = np.arange(1, N_init - 1)
    N = len(Ndim)

    cost_fwd = np.zeros(N)
    cost_bwd = np.zeros(N)

    # Now evaluate each intermediate waypoint (excluding start/end) as a potential docking point.
    for i in range(1, N):
        a = path_smooth[Ndim[i-1]]
        b = path_smooth[Ndim[i]]
        cost_fwd[i] = cost_fwd[i-1] + segment_cost(a, b, cost_type, alpha, beta, v)

    for i in range(N-2, -1, -1):
        a = path_smooth[Ndim[i+1]]
        b = path_smooth[Ndim[i]]
        cost_bwd[i] = cost_bwd[i+1] + segment_cost(a, b, cost_type, alpha, beta, v)

    J_path = cost_fwd + cost_bwd # Check how "central" the point is — you want both drones to travel equal effort/distance ideally
    J_alt = (path_smooth[Ndim, 2] - z_pref)**2 # Prefer docking points that are closer to the receiver’s starting altitude
    J_skew = (cost_fwd - cost_bwd)**2 # Prefer docking points that result in equal travel effort from both drones

    cost_terms = np.vstack((J_path, J_alt, J_skew)).T
    min_vals = cost_terms.min(axis=0)
    max_vals = cost_terms.max(axis=0)
    range_vals = np.where(max_vals - min_vals == 0, 1, max_vals - min_vals)
    norm_costs = (cost_terms - min_vals) / range_vals

    J_all = w_path * norm_costs[:, 0] + w_alt * norm_costs[:, 1] + w_skew * norm_costs[:, 2]
    best_idx_local = np.argmin(J_all)
    docking_cost = J_all[best_idx_local] # How good the docking is
    best_idx = Ndim[best_idx_local]  # Convert to true index in original path (Where in the path it is)
    docking_point = path_smooth[best_idx] # The best point to meet at

    return docking_point, docking_cost, best_idx, path_smooth

# ???
def segment_cost(p1, p2, cost_type, alpha, beta, v):
    delta = p2 - p1
    if cost_type == 'euclidean':
        return np.linalg.norm(delta)
    elif cost_type == 'time':
        return np.linalg.norm(delta) / v
    elif cost_type == 'energy':
        return alpha * np.linalg.norm(delta[:2]) + beta * abs(delta[2])
    else:
        raise ValueError("Unknown cost type")

# --------------------------
# Objective: Coordinates the autonomous docking of two drones (Receiver and Supplier), while avoiding 
# No-Fly Zones (NFZs) and minimizing a cost function by:
# 1). Calling an RRT*-based planner to find a safe efficient docking point
# 2). If that fails, falling back to a failsafe planner that tries progressively larger search boxes
# 3). Generating smooth, obstacle-free flight paths for both drones to reach the docking point
# 4). Returning the docking point, paths, and total cost
# --------------------------
                                             
import numpy as np
from get_docking_rrt_star import get_docking_rrt_star
from get_docking_failsafe import get_docking_failsafe
from smooth_path import smooth_rrt_path

def compute_optimal_docking_path(
    posR, posS, vR, vS, 
    NFZ_centers, NFZ_radii, 
    bounds, cost_type='energy', 
    alpha=1.0, beta=10.0, v=5 
):
    """
    Computes optimal docking point and drone paths using RRT*+ or fallback method.

    Parameters
    ----------
    posR : np.ndarray (3,)
        Initial position of receiver drone in XYZ (Z-up). (m)
    posS : np.ndarray (3,)
        Initial position of supplier drone in XYZ (Z-up). (m)
    vR : float
        Velocity of receiver drone (m/s).
    vS : float
        Velocity of supplier drone (m/s).
    NFZ_centers : np.ndarray (N, 3)
        NFZ center positions in XYZ (Z-up). (m) 
    NFZ_radii : np.ndarray (N,)
        Array of NFZ radii corresponding to each center. (m)
    bounds : dict
        Dictionary with keys 'x', 'y', 'z', each mapping to a [min, max] list.
    cost_type : str, optional
        Cost type for planning: 'euclidean', 'time', or 'energy'. Default is 'energy'.
    alpha : float, optional
        Horizontal cost weight (for 'energy'). Default is 1.0.
    beta : float, optional
        Vertical cost weight (for 'energy'). Default is 10.0.
    v : float, optional
        Constant velocity (for 'time' cost). Default is 5.

    Returns
    -------
    docking_point : np.ndarray (3,)
        Chosen docking point in XYZ coordinates.
    pathR : np.ndarray (N1, 3)
        Smoothed waypoints for receiver to reach docking point. (m)
    pathS : np.ndarray (N2, 3)
        Smoothed waaypoints for supplier to reach docking point. (m)
    docking_cost : float
        Total normalized cost for docking decision.
    """
    # -------------------------------
    # get_docking_rrt_star Objective: 
    # Find a shared docking point, plan a single path that both drones can split and follow, and return the best_idx 
    # so each drone knows where to start/stop on that shared path. If that works: 
    # 1). Separate the two drones' paths at the midpoint, best_idx
    # 2). Add a vertical "z-separation" to keep drones from colliding at docking
    # 3). Smooths both paths
    # -------------------------------
    
    docking_point, docking_cost, best_idx, path_rrt = get_docking_rrt_star(
        posR, posS, NFZ_centers, NFZ_radii, bounds, cost_type, alpha, beta
    )

    # -------------------------------
    # get_docking_rrt_star Objective: 
    # This planner expands the search area gradually (by increasing xmar/y/zmar) ... It tries up to 7 times
    # If successful:
    # 1). Place the receiver slightly above the docking point
    # 2). The supplier goes slightly below
    # If all else fails, it returns “do nothing” paths (i.e., just hover at original positions).
    
    if docking_point is None:
        dx = dy = dz = 1
        visited_keys = set()
        max_attempts = 7
        attempts = 0
        xmar_p = xmar_m = 0
        ymar_p = ymar_m = 0
        zmar_p = zmar_m = 4
        success = False

        while not success and attempts < max_attempts:
            xyz_min = np.minimum(posR, posS) - np.array([xmar_m, ymar_m, zmar_m])
            xyz_max = np.maximum(posR, posS) + np.array([xmar_p, ymar_p, zmar_p])

            bounds = {
                'x': [xyz_min[0], xyz_max[0]],
                'y': [xyz_min[1], xyz_max[1]],
                'z': [xyz_min[2], xyz_max[2]]
            }

            docking_point, docking_cost, cost_terms, visited_keys = get_docking_failsafe(
                posR, posS, vR, vS, NFZ_centers, NFZ_radii, bounds, cost_type,
                alpha, beta, visited_keys
            )

            success = docking_point is not None
            attempts += 1
            xmar_p += 2; xmar_m += 2
            ymar_p += 2; ymar_m += 2
            zmar_p += 1; zmar_m += 1

        if success:
            z_sep = 1.25
            receiver_final = docking_point + np.array([0, 0, z_sep])
            supplier_final = docking_point - np.array([0, 0, z_sep])
            pathR = np.vstack([posR, receiver_final])
            pathS = np.vstack([posS, supplier_final])
        else:
            pathR = np.vstack([posR, posR])
            pathS = np.vstack([posS, posS])
            docking_point = posR * 0  # or None
    else:
        z_sep = 1.25
        receiver_final = docking_point + np.array([0, 0, z_sep])
        supplier_final = docking_point - np.array([0, 0, z_sep])
        pathR = np.vstack([posR, path_rrt[1:best_idx], receiver_final])
        pathS = np.vstack([posS, path_rrt[best_idx+1:][::-1], supplier_final])

        # -------------------------------
        # smooth_rrt_path Objective: 
        # Interpolate points between waypoints
        # Check and avoid NFZ collisions
        # Optimize energy/time efficiency
        # -------------------------------
        
        pathR = smooth_rrt_path(pathR, NFZ_centers, NFZ_radii, cost_type, alpha, beta, v)
        pathS = smooth_rrt_path(pathS, NFZ_centers, NFZ_radii, cost_type, alpha, beta, v)

    return docking_point, pathR, pathS, docking_cost

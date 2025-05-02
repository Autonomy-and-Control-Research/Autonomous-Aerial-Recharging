import numpy as np

def get_docking_failsafe(posR, posS, vR, vS, NFZ_centers, NFZ_radii, bounds, cost_type, alpha, beta, visited_keys):
    dx = dy = dz = 1
    h_clearance = 5
    z_pref = posR[2]
    z_ground, z_ceiling = bounds['z']

    w_path = 1.0
    w_alt = 1.0
    w_skew = 1.0

    x_vals = np.arange(bounds['x'][0], bounds['x'][1] + dx, dx)
    y_vals = np.arange(bounds['y'][0], bounds['y'][1] + dy, dy)
    z_vals = np.arange(bounds['z'][0], bounds['z'][1] + dz, dz)
    X, Y, Z = np.meshgrid(x_vals, y_vals, z_vals, indexing='ij')
    candidates = np.vstack([X.ravel(), Y.ravel(), Z.ravel()]).T

    keys = (X.ravel() * 1e6 + Y.ravel() * 1e3 + Z.ravel()).astype(int)
    mask = ~np.isin(keys, list(visited_keys))
    visited_keys.update(keys[mask])

    candidates = candidates[mask]
    valid_points = []
    cost_terms = []

    for pk in candidates:
        if in_no_fly_zone(pk, NFZ_centers, NFZ_radii):
            continue
        if not has_vertical_clearance(pk, NFZ_centers, NFZ_radii, h_clearance, z_ground, z_ceiling):
            continue
        if crosses_nfz_segment(posR, pk, NFZ_centers, NFZ_radii) or crosses_nfz_segment(posS, pk, NFZ_centers, NFZ_radii):
            continue

        dR = np.linalg.norm(pk - posR)
        dS = np.linalg.norm(pk - posS)

        if cost_type == 'euclidean':
            T_R, T_S = dR, dS
        elif cost_type == 'time':
            T_R, T_S = dR / vR, dS / vS
        elif cost_type == 'energy':
            T_R = alpha * np.linalg.norm(pk[:2] - posR[:2]) + beta * abs(pk[2] - posR[2])
            T_S = alpha * np.linalg.norm(pk[:2] - posS[:2]) + beta * abs(pk[2] - posS[2])
        else:
            raise ValueError("Unknown cost type")

        J_alt = (pk[2] - z_pref)**2
        J_skew = abs(T_R - T_S)
        
        valid_points.append(pk)
        cost_terms.append([T_R + T_S, J_alt, J_skew])

    if not cost_terms:
        return None, float('inf'), {}, visited_keys

    cost_terms = np.array(cost_terms)
    min_vals = cost_terms.min(axis=0)
    max_vals = cost_terms.max(axis=0)
    range_vals = np.where(max_vals - min_vals == 0, 1, max_vals - min_vals)
    norm_costs = (cost_terms - min_vals) / range_vals

    J_all = w_path * norm_costs[:, 0] + w_alt * norm_costs[:, 1] + w_skew * norm_costs[:, 2]
    idx = np.argmin(J_all)

    best_point = valid_points[idx]
    best_cost = J_all[idx]
    best_terms = {
        'T_R': cost_terms[idx, 0] / 2,
        'T_S': cost_terms[idx, 0] / 2,
        'J_alt': cost_terms[idx, 1],
        'J_skew': cost_terms[idx, 2]
    }

    return np.array(best_point), best_cost, best_terms, visited_keys

def in_no_fly_zone(p, centers, radii):
    for c, r in zip(centers, radii):
        if np.linalg.norm(p - c) <= r:
            return True
    return False

def has_vertical_clearance(p, centers, radii, h_clear, z_min, z_max):
    if p[2] < z_min + 1 or p[2] > z_max - 1:
        return False
    for c, r in zip(centers, radii):
        horiz_dist = np.linalg.norm(p[:2] - c[:2])
        if horiz_dist < r:
            vert_clear = abs(p[2] - c[2])
            if vert_clear < h_clear:
                return False
    return True

# def crosses_nfz_segment(p1, p2, centers, radii):
#     ab = p2 - p1
#     for c, r in zip(centers, radii):
#         ac = c - p1
#         t = np.clip(np.dot(ac, ab) / np.dot(ab, ab), 0, 1)
#         proj = p1 + t * ab
#         if np.linalg.norm(proj - c) < r:
#             return True
#     return False

def crosses_nfz_segment(p1, p2, centers, radii):
    ab = p2 - p1
    if np.allclose(ab, 0):
        return False  # no segment to check

    for c, r in zip(centers, radii):
        ac = c - p1
        denom = np.dot(ab, ab)
        if denom == 0:
            continue  # skip degenerate segment
        t = np.clip(np.dot(ac, ab) / denom, 0, 1)
        proj = p1 + t * ab
        if np.linalg.norm(proj - c) < r:
            return True
    return False
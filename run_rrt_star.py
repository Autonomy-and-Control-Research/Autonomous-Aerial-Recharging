import numpy as np
# from smooth_path import smooth_rrt_path

def run_rrt_star_3d_plus(start_pos, goal_pos, NFZ_centers, NFZ_radii, bounds, cost_type, alpha, beta):
    max_iter = 2000
    step_size = 0.3
    goal_threshold = 2.0
    search_radius = 3.0
    goal_bias = 0.1
    v = 5  # used for time-based cost

    nodes = [start_pos]
    parents = [-1]
    costs = [0.0]
    success = False

    for _ in range(max_iter):
        if np.random.rand() < goal_bias and is_line_of_sight(start_pos, goal_pos, NFZ_centers, NFZ_radii):
            rand_pt = goal_pos
        else:
            rand_pt = np.array([
                np.random.uniform(*bounds['x']),
                np.random.uniform(*bounds['y']),
                np.random.uniform(*bounds['z'])
            ])

        dists = np.linalg.norm(np.array(nodes) - rand_pt, axis=1)
        nearest_idx = np.argmin(dists)
        nearest = nodes[nearest_idx]

        direction = rand_pt - nearest
        direction /= np.linalg.norm(direction)
        new_pt = nearest + step_size * direction

        if in_no_fly_zone(new_pt, NFZ_centers, NFZ_radii):
            continue

        neighbor_indices = [i for i, node in enumerate(nodes) if np.linalg.norm(node - new_pt) <= search_radius]

        min_cost = costs[nearest_idx] + edge_cost(nearest, new_pt, cost_type, alpha, beta, v)
        best_parent = nearest_idx

        for idx in neighbor_indices:
            cand_cost = costs[idx] + edge_cost(nodes[idx], new_pt, cost_type, alpha, beta, v)
            mid_point = (nodes[idx] + new_pt) / 2
            if cand_cost < min_cost and not in_no_fly_zone(mid_point, NFZ_centers, NFZ_radii):
                min_cost = cand_cost
                best_parent = idx

        nodes.append(new_pt)
        parents.append(best_parent)
        costs.append(min_cost)

        for idx in neighbor_indices:
            rew_cost = min_cost + edge_cost(new_pt, nodes[idx], cost_type, alpha, beta, v)
            if rew_cost < costs[idx]:
                parents[idx] = len(nodes) - 1
                costs[idx] = rew_cost

        if np.linalg.norm(new_pt - goal_pos) < goal_threshold:
            success = True
            break

    if not success:
        return None, float('inf'), False

    nodes = np.array(nodes)
    goal_idx = np.argmin(np.linalg.norm(nodes - goal_pos, axis=1))
    cost = costs[goal_idx]
    path = [goal_pos]
    idx = goal_idx

    while idx != -1:
        path.insert(0, nodes[idx])
        idx = parents[idx]

    path = np.array(path)
    # path = smooth_rrt_path(path, NFZ_centers, NFZ_radii, cost_type, alpha, beta, v)
    return path, cost, True

def in_no_fly_zone(p, centers, radii):
    for c, r in zip(centers, radii):
        if np.linalg.norm(p - c) <= r:
            return True
    return False

def is_line_of_sight(p1, p2, centers, radii):
    for c, r in zip(centers, radii):
        if point_to_segment_distance(c, p1, p2) < r:
            return False
    return True

def point_to_segment_distance(c, a, b):
    ab = b - a
    t = np.clip(np.dot(c - a, ab) / np.dot(ab, ab), 0, 1)
    proj = a + t * ab
    return np.linalg.norm(c - proj)

def edge_cost(p1, p2, cost_type, alpha, beta, v):
    delta = p2 - p1
    if cost_type == 'euclidean':
        return np.linalg.norm(delta)
    elif cost_type == 'time':
        return np.linalg.norm(delta) / v
    elif cost_type == 'energy':
        return alpha * np.linalg.norm(delta[:2]) + beta * abs(delta[2])
    else:
        raise ValueError("Unknown cost type")

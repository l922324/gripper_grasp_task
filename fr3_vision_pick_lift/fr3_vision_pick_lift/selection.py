"""Reject unsuitable grasps without snapping or replacing learned poses."""
import math
import numpy as np
from scipy.spatial.transform import Rotation


def select_candidates(grasps, scores, center, minimum_score=0.5, max_tilt_degrees=5.,
                      base_to_flange=None, tip_depth=0.163, table_top=0.02, grasp_center_depth=0.145):
    grasps, scores = np.asarray(grasps), np.asarray(scores)
    if grasps.shape != (len(scores), 4, 4) or scores.ndim != 1:
        raise ValueError('GraspGenX response must contain Kx4x4 poses and K scores')
    mount = np.eye(4) if base_to_flange is None else np.asarray(base_to_flange)
    if mount.shape != (4,4) or not np.isfinite(mount).all():
        raise ValueError('Invalid model-base to flange calibration')
    if not np.allclose(mount[:3,:3].T @ mount[:3,:3], np.eye(3), atol=1e-6):
        raise ValueError('Mount calibration rotation is not rigid')
    candidates = []
    # Conservative open-gripper envelope, including the tips when closing.
    corners = np.array([[x, y, z] for x in (-0.070, 0.070)
                        for y in (-0.020, 0.020) for z in (0., tip_depth)])
    for index in np.argsort(-scores):
        pose = grasps[index]
        score = float(scores[index])
        if not np.isfinite(pose).all() or not math.isfinite(score) or score < minimum_score:
            continue
        if not np.allclose(pose[3], [0., 0., 0., 1.], atol=1e-5):
            continue
        r = pose[:3, :3]
        if not np.allclose(r.T @ r, np.eye(3), atol=1e-3) or abs(np.linalg.det(r)-1.) > 1e-3:
            continue
        flange = pose @ mount
        r, position = flange[:3, :3], flange[:3, 3]
        if -r[2, 2] < math.cos(math.radians(max_tilt_degrees)):
            continue
        if np.linalg.norm(position[:2] - center[:2]) > 0.035:
            continue
        grasp_center = position + r @ [0., 0., grasp_center_depth]
        if np.linalg.norm(grasp_center - center) > 0.035:
            continue
        world_corners = corners @ r.T + position
        if world_corners[:, 2].min() < table_top + 0.001:
            continue
        candidates.append((score, flange))
    return candidates


def mount_matrix(xyz, rpy):
    if (np.shape(xyz) != (3,) or np.shape(rpy) != (3,) or
            not np.isfinite([*xyz, *rpy]).all()):
        raise ValueError('Mount xyz/rpy must contain three finite values each')
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    result[:3, 3] = xyz
    return result

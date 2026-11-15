"""RGB point-cloud segmentation and box pose estimation, with no spawn-pose input."""
import math
import numpy as np
from scipy.spatial import ConvexHull, cKDTree
from scipy.spatial.transform import Rotation


def decode_cloud(message):
    """Decode organized/padded/endian PointCloud2 messages without sensor_msgs_py."""
    endian = '>' if message.is_bigendian else '<'
    types = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
    fields = {field.name: field for field in message.fields}
    if not {'x', 'y', 'z'} <= fields.keys():
        raise ValueError('Point cloud has no XYZ fields')
    names, formats, offsets = [], [], []
    for name, field in fields.items():
        if field.count != 1 or field.datatype not in types:
            continue
        names.append(name)
        formats.append(endian + types[field.datatype])
        offsets.append(field.offset)
    dtype = np.dtype(dict(names=names, formats=formats, offsets=offsets,
                         itemsize=message.point_step))
    if len(message.data) < message.row_step * message.height:
        raise ValueError('Truncated point cloud')
    organized = np.ndarray((message.height, message.width), dtype=dtype,
                           buffer=message.data,
                           strides=(message.row_step, message.point_step))
    xyz = np.column_stack([organized[name].reshape(-1) for name in ('x', 'y', 'z')])
    packed = 'rgb' if 'rgb' in fields else 'rgba' if 'rgba' in fields else None
    if packed:
        if fields[packed].datatype not in (6, 7):
            raise ValueError('Packed RGB must be a 32-bit field')
        # Interpret the color bits, rather than numerically casting a float RGB field.
        bits = np.ascontiguousarray(organized[packed]).view(endian + 'u4').reshape(-1)
        rgb = np.column_stack([(bits >> shift) & 255 for shift in (16, 8, 0)])
    elif {'r', 'g', 'b'} <= fields.keys():
        rgb = np.column_stack([organized[name].reshape(-1) for name in ('r', 'g', 'b')])
    else:
        raise ValueError('RGB is required to identify the green block')
    valid = np.isfinite(xyz).all(axis=1)
    return xyz[valid].astype(np.float64), rgb[valid].astype(np.uint8)


def transform_points(points, transform):
    t = transform.transform
    q = t.rotation
    rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    return points @ rotation.T + [t.translation.x, t.translation.y, t.translation.z]


def green_points(xyz, rgb):
    red, green, blue = rgb.astype(float).T
    mask = (green > 45) & (green > 1.5 * red + 10) & (green > 1.5 * blue + 10)
    return xyz[mask]


def crop_table(points, position, size, quaternion, block_height):
    """Crop in the configured table frame, never around the spawn pose."""
    p = np.asarray(points)
    local = (p - position) @ Rotation.from_quat(quaternion).as_matrix()
    half = np.asarray(size)/2
    mask = ((np.abs(local[:, 0]) < half[0]) & (np.abs(local[:, 1]) < half[1]) &
            (local[:, 2] > half[2] + 0.003) &
            (local[:, 2] < half[2] + block_height + 0.04))
    return p[mask]


def fuse_object_cloud(clouds, voxel_size=0.002, minimum=100):
    if len(clouds) != 2 or any(len(cloud) == 0 for cloud in clouds):
        raise ValueError('Two nonempty camera observations are required')
    points = np.concatenate(clouds)
    _, indices = np.unique(np.floor(points / voxel_size).astype(np.int64), axis=0,
                           return_index=True)
    points = points[indices]
    # Select a connected component so isolated green pixels cannot distort the hull.
    tree = cKDTree(points)
    unseen = set(range(len(points)))
    largest = []
    while unseen:
        component, stack = [], [unseen.pop()]
        while stack:
            index = stack.pop()
            component.append(index)
            neighbors = unseen.intersection(tree.query_ball_point(points[index], 0.009))
            unseen.difference_update(neighbors)
            stack.extend(neighbors)
        if len(component) > len(largest):
            largest = component
    points = points[largest]
    if len(points) < minimum:
        raise ValueError(f'Only {len(points)} fused object points; need {minimum}')
    return points.astype(np.float32)


def estimate_block(points, size=0.04, table_top=0.02):
    """Fit a minimum-area XY rectangle; a square's yaw is observable modulo pi/2."""
    size = np.repeat(float(size), 3) if np.ndim(size) == 0 else np.asarray(size)
    points = np.asarray(points)
    if len(points) < 20 or not np.isfinite(points).all():
        raise ValueError('Insufficient finite points for block pose estimation')
    hull = points[ConvexHull(points[:, :2]).vertices, :2]
    edges = np.roll(hull, -1, axis=0) - hull
    angles = np.arctan2(edges[:, 1], edges[:, 0])
    best = None
    for angle in angles:
        c, s = math.cos(angle), math.sin(angle)
        basis = np.array([[c, -s], [s, c]])
        aligned = hull @ basis
        lower, upper = aligned.min(axis=0), aligned.max(axis=0)
        area = np.prod(upper - lower)
        if best is None or area < best[0]:
            best = (area, basis @ ((lower + upper) / 2), upper - lower, angle)
    _, center_xy, extents, angle = best
    # Resolve rectangle-axis ambiguity against the configured footprint.
    if np.linalg.norm(extents[::-1] - size[:2]) < np.linalg.norm(extents - size[:2]):
        extents = extents[::-1]
        angle += math.pi/2
    if (extents < size[:2] * 0.65).any() or (extents > size[:2] * 1.4).any():
        raise ValueError(f'Observed footprint {extents} does not match the configured {size} m block')
    # Use the observed top face, not the biased centroid of visible surfaces.
    center_z = float(np.quantile(points[:, 2], 0.95) - size[2] / 2)
    if abs(center_z - (table_top + size[2] / 2)) > 0.01:
        raise ValueError('Detected object height is inconsistent with the tabletop')
    period = math.pi/2 if np.isclose(size[0], size[1]) else math.pi
    yaw = (angle + period/2) % period - period/2
    return np.array([*center_xy, center_z]), yaw, extents

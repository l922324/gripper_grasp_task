"""Client for the documented GraspGenX ZMQ/msgpack protocol; no local model fallback."""
import numpy as np


def infer_grasps(points, endpoint, timeout=60., num_grasps=512):
    try:
        import zmq
        import msgpack
        import msgpack_numpy
    except ImportError as exc:
        raise RuntimeError(
            'GraspGenX client dependencies are missing. Install pyzmq, msgpack and '
            'msgpack-numpy in the ROS Python environment; see README.md.') from exc
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError('Inference requires finite Nx3 point cloud coordinates in metres')
    request = dict(action='infer', point_cloud=points, gripper_name='robotiq_2f_85',
                   num_grasps=int(num_grasps), grasp_threshold=-1.0,
                   topk_num_grasps=int(num_grasps))
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.SNDTIMEO, int(timeout * 1000))
    socket.setsockopt(zmq.RCVTIMEO, int(timeout * 1000))
    try:
        socket.connect(endpoint)
        socket.send(msgpack.packb(request, default=msgpack_numpy.encode, use_bin_type=True))
        response = msgpack.unpackb(socket.recv(), object_hook=msgpack_numpy.decode,
                                   raw=False, strict_map_key=False)
    except zmq.Again as exc:
        raise RuntimeError(f'GraspGenX timed out at {endpoint}; no arm motion was started') from exc
    finally:
        socket.close()
        context.term()
    if not isinstance(response, dict) or 'error' in response:
        raise RuntimeError(f'GraspGenX error: {response}')
    if response.get('gripper_name', 'robotiq_2f_85') != 'robotiq_2f_85':
        raise RuntimeError('Inference server returned a different gripper embodiment')
    return np.asarray(response['grasps'], dtype=float), np.asarray(response['confidences'], dtype=float)

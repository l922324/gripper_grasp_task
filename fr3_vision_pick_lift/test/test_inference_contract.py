"""Protocol fixtures only: these tests do not generate learned grasps."""
import threading
import uuid
import numpy as np
import pytest

from fr3_vision_pick_lift.inference import infer_grasps


def test_documented_wire_contract(monkeypatch):
    zmq = pytest.importorskip('zmq')
    msgpack = pytest.importorskip('msgpack')
    codec = pytest.importorskip('msgpack_numpy')
    context = zmq.Context()
    endpoint = 'inproc://graspgen-contract-' + uuid.uuid4().hex
    reply = context.socket(zmq.REP)
    reply.bind(endpoint)
    requests = []
    pose = np.eye(4, dtype=np.float32)[None]

    def server():
        try:
            request = msgpack.unpackb(reply.recv(), object_hook=codec.decode, raw=False)
            requests.append(request)
            reply.send(msgpack.packb(dict(grasps=pose, confidences=np.array([.9]),
                                          gripper_name='robotiq_2f_85'),
                                    default=codec.encode, use_bin_type=True))
        finally:
            reply.close(0)

    thread = threading.Thread(target=server)
    thread.start()
    monkeypatch.setattr(zmq, 'Context', lambda: context)
    points = np.ones((100, 3), dtype=np.float32)
    grasps, scores = infer_grasps(points, endpoint, timeout=2.)
    thread.join(timeout=2.)
    assert not thread.is_alive()
    assert requests[0]['action'] == 'infer'
    assert requests[0]['gripper_name'] == 'robotiq_2f_85'
    assert np.array_equal(requests[0]['point_cloud'], points)
    assert np.array_equal(grasps, pose)
    assert scores[0] == pytest.approx(.9)


def test_unavailable_server_times_out_without_fallback():
    pytest.importorskip('zmq')
    pytest.importorskip('msgpack_numpy')
    with pytest.raises(RuntimeError, match='timed out'):
        infer_grasps(np.ones((100,3)), 'inproc://no-server-' + uuid.uuid4().hex, timeout=.02)

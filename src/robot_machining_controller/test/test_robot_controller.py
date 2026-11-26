import pytest

fanuc_msgs = pytest.importorskip('fanuc_msgs.msg')  # needs the FANUC driver underlay (~/ws_fanuc)
rclpy = pytest.importorskip('rclpy')

from robot_machining_controller.robot_controller import RobotController  # noqa: E402


@pytest.fixture
def node():
    rclpy.init()
    n = RobotController()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def ok_status():
    return fanuc_msgs.RobotStatus(motion_possible=True)


def test_status_gate(node):
    now = node.get_clock().now()
    assert 'no connection' in node.robot_problem(now)  # nothing from the driver yet
    node.connection = (fanuc_msgs.ConnectionStatus(is_connected=True), now)
    assert 'no robot status' in node.robot_problem(now)
    for bad in ({'in_error': True}, {'e_stopped': True}, {'tp_enabled': True}, {'motion_possible': False}):
        node.status = (fanuc_msgs.RobotStatus(**{'motion_possible': True, **bad}), now)
        assert 'robot status' in node.robot_problem(now), bad
    node.status = (ok_status(), now)
    assert node.robot_problem(now).startswith('no TF')  # status fine: next gate (Servo frame) reached
    old = now - rclpy.duration.Duration(seconds=1.0)
    node.status = (ok_status(), old)
    assert 'no robot status' in node.robot_problem(now)  # stale status


def test_mock_skips_status_only_while_none_arrives(node):
    node.require_robot_status = False
    now = node.get_clock().now()
    assert node.robot_problem(now).startswith('no TF')  # status skipped
    node.connection = (fanuc_msgs.ConnectionStatus(is_connected=False), now)
    assert 'no connection' in node.robot_problem(now)  # a real driver status is always enforced

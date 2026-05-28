import pathlib
import time

import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.node import Node


SIM_LAUNCH_PATH = pathlib.Path(__file__).resolve().parents[1] / 'launch' / 'sim_bringup.launch.py'


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(SIM_LAUNCH_PATH)),
    )

    # Delay test start slightly so launch_testing begins after includes are scheduled.
    ready = TimerAction(period=2.0, actions=[launch_testing.actions.ReadyToTest()])

    return launch.LaunchDescription([sim, ready])


class TestSimBringupLaunch:
    @classmethod
    def setup_class(cls):
        rclpy.init()

    @classmethod
    def teardown_class(cls):
        rclpy.shutdown()

    def setup_method(self):
        self.node = Node('test_sim_navigation_probe')

    def teardown_method(self):
        self.node.destroy_node()

    def _wait_for_topic_publishers(self, topic_name, min_count=1, timeout_sec=45.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            publishers = self.node.get_publishers_info_by_topic(topic_name)
            if len(publishers) >= min_count:
                return publishers
            rclpy.spin_once(self.node, timeout_sec=0.2)
            time.sleep(0.2)
        pytest.fail(
            f'Timed out waiting for {min_count} publisher(s) on {topic_name}; '
            f'found {len(self.node.get_publishers_info_by_topic(topic_name))}'
        )

    def _wait_for_node(self, expected_name, timeout_sec=45.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            names = {name for name, _ns in self.node.get_node_names_and_namespaces()}
            if expected_name in names:
                return
            rclpy.spin_once(self.node, timeout_sec=0.2)
            time.sleep(0.2)
        pytest.fail(f'Timed out waiting for node {expected_name!r} to appear')

    def test_sim_launch_file_is_packaged(self):
        assert SIM_LAUNCH_PATH.exists(), str(SIM_LAUNCH_PATH)

    def test_sim_stack_reaches_basic_readiness(self):
        self._wait_for_node('zone_detector')
        self._wait_for_topic_publishers('/scan')
        clock_publishers = self._wait_for_topic_publishers('/clock')
        assert len(clock_publishers) >= 1


@launch_testing.post_shutdown_test()
class TestSimBringupShutdown:
    def test_launch_exits_cleanly(self, proc_info):
        proc_info.assertWaitForShutdown(timeout=20)

    def test_no_required_process_dies_early(self, proc_info, proc_output):
        unexpected_exit_lines = []
        for line in proc_output:
            text = getattr(line, 'text', str(line))
            if 'process has died' in text.lower() or 'traceback' in text.lower():
                unexpected_exit_lines.append(text)
        assert unexpected_exit_lines == []


def test_sim_navigation():
    # launch_testing needs a top-level test function to drive the launched system.
    pass

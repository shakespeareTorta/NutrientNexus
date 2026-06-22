from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'my_turtlebot3_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bulut Tekinsen',
    maintainer_email='bulut.tekinsen@gmail.com',
    description='Digital-twin controller for an autonomous TurtleBot3 '
                'precision-agriculture robot (Nutrient Nexus): patrols field '
                'zones, monitors soil telemetry, and applies targeted '
                'irrigation and fertilisation.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'navigation_executor_node = '
            'my_turtlebot3_controller.navigation.NavigationExecutorNode:main',
            'field_sensor_mock_node = my_turtlebot3_controller.sensor.FieldSensorMockNode:main',
            'weather_adapter_node = my_turtlebot3_controller.sensor.WeatherAdapterNode:main',
            'crop_decision_node = my_turtlebot3_controller.algorithm.CropDecisionNode:main',
            'safety_stop_node = my_turtlebot3_controller.navigation.SafetyStopNode:main',
            'zone_detector_node = my_turtlebot3_controller.navigation.ZoneDetectorNode:main',
            'robot_resource_node = my_turtlebot3_controller.RobotResourceNode:main',
            'dashboard_node = my_turtlebot3_controller.dashboard.DashboardNode:main',
            'sustainability_audit_node = '
            'my_turtlebot3_controller.audit.SustainabilityAuditNode:main',
            'twin_supervisor_node = my_turtlebot3_controller.twin.TwinSupervisorNode:main',
            'system_monitor_node = my_turtlebot3_controller.SystemMonitorNode:main',
            'zone_visualizer_node = '
            'my_turtlebot3_controller.visualization.ZoneVisualizerNode:main',
            'ground_truth_localization = '
            'my_turtlebot3_controller.localization.GroundTruthLocalizationNode:main',
        ],
    },
)

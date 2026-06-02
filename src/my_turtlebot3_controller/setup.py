from setuptools import find_packages, setup
import os
from glob import glob
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
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'move_turtlebot = my_turtlebot3_controller.navigation.MoveTurtleBot:main',
            'cmd_vel_relay_node = my_turtlebot3_controller.CmdVelRelay:main',
            'DecisionNode = my_turtlebot3_controller.algorithm.DecisionNode:main',
            'BinSensorMockNode = my_turtlebot3_controller.sensor.BinSensorMockNode:main',
            'navigation_executor_node = my_turtlebot3_controller.navigation.NavigationExecutorNode:main',
            'odom_node = my_turtlebot3_controller.navigation.odometry.OdomToGazeboPoseNode:main',
            'navigation_node = my_turtlebot3_controller.navigation.NavigationNode:main',
            'field_sensor_mock_node = my_turtlebot3_controller.sensor.FieldSensorMockNode:main',
            'crop_decision_node = my_turtlebot3_controller.algorithm.CropDecisionNode:main',
            'safety_stop_node = my_turtlebot3_controller.navigation.SafetyStopNode:main',
            'twin_safety_node = my_turtlebot3_controller.twin.TwinSafetyNode:main',
            'zone_detector_node = my_turtlebot3_controller.navigation.ZoneDetectorNode:main',
            'robot_resource_node = my_turtlebot3_controller.RobotResourceNode:main',
            'dashboard_node = my_turtlebot3_controller.dashboard.DashboardNode:main',
            'sustainability_audit_node = my_turtlebot3_controller.audit.SustainabilityAuditNode:main',
            'twin_supervisor_node = my_turtlebot3_controller.twin.TwinSupervisorNode:main',
            'odom_to_tf_node = my_turtlebot3_controller.navigation.odometry.OdomToTFNode:main',
        ],
    },
)

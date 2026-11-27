import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

PKG = 'robot_machining_controller'
ROBOT_MODEL = 'crx10ia_l'  # CRX-10iA/L, URDF name in fanuc_description


def servo(context):
    """MoveIt Servo on the same robot model as the driver (fanuc_moveit_config srdf + kinematics)."""
    arg = LaunchConfiguration
    urdf = os.path.join(get_package_share_directory('fanuc_hardware_interface'), 'robot', f'{ROBOT_MODEL}.urdf.xacro')
    gpio = os.path.join(get_package_share_directory('fanuc_hardware_interface'), 'config', 'example_gpio_config.yaml')
    moveit_config = (
        MoveItConfigsBuilder(ROBOT_MODEL, package_name='fanuc_moveit_config')
        .robot_description(file_path=urdf, mappings={
            'robot_series': 'crx', 'robot_model': ROBOT_MODEL, 'prefix': '', 'gpio_configuration': gpio,
            'robot_ip': arg('robot_ip').perform(context), 'use_mock': arg('use_mock').perform(context)})
        .robot_description_semantic(file_path=f'srdf/{ROBOT_MODEL}.srdf')
        .planning_pipelines(pipelines=['ompl'])  # default list includes pilz, whose config FANUC does not ship
        .to_moveit_configs())
    servo_params = {'moveit_servo': ParameterBuilder(PKG).yaml('config/servo.yaml').to_dict()}
    return [Node(package='moveit_servo', executable='servo_node', name='servo_node', output='screen',
                 parameters=[servo_params, moveit_config.robot_description,
                             moveit_config.robot_description_semantic,
                             moveit_config.robot_description_kinematics, moveit_config.joint_limits])]


def mock_home(context):
    """Mock only: the mock starts at all zeros, a wrist singularity where Servo does not move."""
    if LaunchConfiguration('use_mock').perform(context).lower() != 'true':
        return []
    positions = [float(a) for a in LaunchConfiguration('mock_home').perform(context).split()]
    return [Node(package=PKG, executable='mock_home', output='screen', parameters=[{'positions_deg': positions}])]


def generate_launch_description():
    """FANUC ROS 2 driver (ros2_control, forward_position_controller, gpio status) + MoveIt Servo.

    use_mock:=true runs the driver's mock hardware: no robot, nothing moves. With use_mock:=false the robot must be
    in AUTO, alarms cleared, teach pendant off (FANUC driver quick start). Start follow_marker.launch.py separately.
    """
    driver = os.path.join(get_package_share_directory('fanuc_forward_command'), 'launch',
                          'fanuc_forward_command.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('use_mock', default_value='true',
                              description='true = FANUC mock hardware; false = the real robot at robot_ip'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.100'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('mock_home', default_value='-50.5 25.0 -40.0 -118.0 -61.0 -13.0',
                              description='use_mock only: start joint pose J1..J6 [deg, pendant values]'),
        # ros2_control node + robot_state_publisher (TF base_link -> ... -> fanuc_flange) + controller spawners:
        # joint_state_broadcaster, fanuc_gpio_controller (connection / robot status), forward_position_controller
        IncludeLaunchDescription(driver, launch_arguments={
            'robot_model': ROBOT_MODEL, 'use_mock': LaunchConfiguration('use_mock'),
            'robot_ip': LaunchConfiguration('robot_ip'), 'launch_rviz': LaunchConfiguration('launch_rviz')}.items()),
        OpaqueFunction(function=servo),
        OpaqueFunction(function=mock_home),
    ])

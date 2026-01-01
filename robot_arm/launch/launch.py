# launch/view_robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = FindPackageShare(package='robot_arm') # Your package name

    # 1. robot_state_publisher (publishes TF2 transforms)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[str(pkg_share.find('robot_arm')) + '/urdf/model.urdf'] # Path to URDF
    )

    # 2. joint_state_publisher (for GUI control)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

     # 2. joint_state_publisher (for GUI control)
    joint_state_publisher_node = Node(
        package='robot_arm',
        executable='state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    # 3. RViz2 node (to visualize)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(pkg_share.find('robot_arm')) + '/urdf/model.rviz'] # Optional RViz config
    )

    set_target_node = Node(
        package='robot_arm',
        executable='set_target',
        name='set_target',
        output='screen',
        prefix='xterm -hold -e',
    )
        
    start_gyro_node = Node(
        package='robot_arm',
        executable='start_gyro',
        name='gyro_sensor',
        output='screen',
    )

    get_pos_node = Node(
        package='robot_arm',
        executable='get_pos',
        name='forward_kin',
        output='screen',
    )

    get_angles_node = Node(
        package='robot_arm',
        executable='inverse_kin',
        name='inverse_kin',
    )

    start_motors_node = Node(
        package='robot_arm',
        executable='start_motors',
        name='start_motors',
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        set_target_node,
        start_gyro_node,
        get_pos_node,
        get_angles_node,
        start_motors_node,
        rviz_node
    ])

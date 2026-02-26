from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('linebot')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    models_path = os.path.join(pkg_share, 'models')
    gz_resource_path = models_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    ign_resource_path = models_path + ':' + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')

    set_gz_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path
    )

    set_ign_resource = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=ign_resource_path
    )

    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, 'worlds', 'world.sdf'),
        description='Full path to world SDF'
    )

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        # -r = run immediately
        launch_arguments={'gz_args': [world, ' -r']}.items(),
    )

    # Bridge command topic one-way: ROS -> Gazebo.
    # Use relative topic name to match the model plugin setting.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            'cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
    )

    controller = Node(
        package='linebot',
        executable='linebot',
        output='screen',
)

    return LaunchDescription([
        world_arg,
        set_gz_resource,
        set_ign_resource,
        gz,
        bridge,
        controller,
    ])
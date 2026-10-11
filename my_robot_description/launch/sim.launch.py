import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file to spin up the Gazebo simulation environment,
    spawn the robot model, and initialize perception and behavior nodes.
    """

    # 1. Locate package directories
    pkg_description = get_package_share_directory('my_robot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_controller = get_package_share_directory('my_robot_controller')

    # 2. Read URDF file containing robot geometry and joint structure
    urdf_file = os.path.join(pkg_description, 'urdf', 'robot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    # 3. Node: Robot State Publisher (publishes TF transform tree and robot state)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )

    # 4. Include standard Gazebo launcher with custom world file
    world_file = os.path.join(pkg_description, 'worlds', 'test_world.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items(),
    )

    # 5. Node: Spawn robot model into the Gazebo simulation world
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen',
    )

    # 6. Locate YAML parameter file for nodes
    config_file = os.path.join(pkg_controller, 'config', 'vision_params.yaml')

    # 7. Node: Computer Vision tracker node ("Eyes")
    vision_tracker_node = Node(
        package='my_robot_controller',
        executable='vision_node',
        name='vision_tracker_node',
        output='screen',
        parameters=[config_file],  # Load tuning parameters from YAML
    )

    # 8. Node: Finite State Machine behavior controller ("Brain")
    behavior_node = Node(
        package='my_robot_controller',
        executable='behavior_node',
        name='behavior_node',
        output='screen',
        parameters=[config_file],  # Load state machine parameters from YAML
    )

    # Return complete launch description
    return LaunchDescription([
        robot_state_publisher_node,
        gazebo,
        spawn_entity_node,
        vision_tracker_node,
        behavior_node,
    ])
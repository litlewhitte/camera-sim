import launch
import launch.event_handlers
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.parameter_descriptions


def generate_launch_description():
    # 获取功能包的share路径
    urdf_package_path = get_package_share_directory('fishbot_description')
    default_xacro_path = os.path.join(urdf_package_path,'urdf','fishbot/fishbot.urdf.xacro')
    default_rviz_config_path = os.path.join(urdf_package_path,'config','display_robot_model.rviz')
    default_gazebo_world_path = os.path.join(urdf_package_path,'world/mp3d1','mp3d1.world')
    # 申明一个urdf的参数，方便修改
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_xacro_path)
    )
    # 通过文件路径获取内容，并转换成参数
    substitutions_command_result = launch.substitutions.Command(['xacro ',launch.substitutions.LaunchConfiguration('model')])
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result,value_type=str)
    
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description_value}]
    )
    
    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'),'/launch','/gazebo.launch.py']
        ),
        launch_arguments=[('world',default_gazebo_world_path),('verbose','true')]
    )
    
    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','/robot_description','-entity','fishbot']
    )
    
    # action_joint_state_publisher = launch_ros.actions.Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher'
    # )
    
    action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d' , default_rviz_config_path]
    )
    
    action_load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'configure','fishbot_joint_state_broadcaster'],
        output='screen'
    )

    action_load_fishbot_effort_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'configure','fishbot_effort_controller'], 
        output='screen')
    
    action_load_fishbot_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'configure','fishbot_diff_drive_controller'], 
        output='screen')

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_launch_gazebo,
        action_spawn_entity,
        # action_joint_state_publisher,
        # action_rviz_node
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=action_spawn_entity,
        #         on_exit=[action_load_joint_state_controller],
        #     )
        # ),
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=action_load_joint_state_controller,
        #         on_exit=[action_load_fishbot_diff_drive_controller],
        #     )
        # ),
        
    ])
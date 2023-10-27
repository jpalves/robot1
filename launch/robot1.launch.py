import os

from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.actions import (
    GroupAction,
    SetEnvironmentVariable,
)
from launch_ros.actions import PushRosNamespace
import xacro

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('robot1')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    slam          = LaunchConfiguration("slam")
    namespace     = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    map = LaunchConfiguration("map")
    use_sim_time  = LaunchConfiguration("use_sim_time")
    params_file   = LaunchConfiguration("params_file")
    default_nav_to_pose_bt_xml = LaunchConfiguration("default_nav_to_pose_bt_xml")
    autostart     = LaunchConfiguration("autostart")
    show_gz_lidar = LaunchConfiguration("show_gz_lidar")
    
    rviz_config_file    = LaunchConfiguration("rviz_config_file")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")
    use_tags = LaunchConfiguration("use_tags")
    #...............................................................................
    nav = get_package_share_directory("nav2_bringup")
    apriltag_ros_dir = get_package_share_directory("apriltag_ros")
    #apriltag_ros_launch_dir = os.path.join(apriltag_ros_dir, "launch")
    image_topic = "color/image_raw"
    camera_name = "color"
    #-------------------------------------------------------------------------------
    smoothner_dir = get_package_share_directory('kobuki_velocity_smoother')
    urdf = os.path.join(bringup_dir, "urdf", "turtlebot3_waffle.urdf")
    doc = xacro.parse(open(urdf))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()} # type: ignore

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    
    declare_namespace_cmd = DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace")
    
    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="True",
        description="Whether to apply a namespace to the navigation stack",
    )
    
    declare_show_gz_lidar = DeclareLaunchArgument(
        "show_gz_lidar",
        default_value="False",
        description="Whether to apply a namespace to the navigation stack",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam", default_value="True", description="Whether run a SLAM"
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="True", description="Use simulation (Gazebo) clock if true"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(bringup_dir, "maps", "my_map.yaml"),
        description="Full path to map file to load",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_bt_xml_cmd = DeclareLaunchArgument(
        "default_nav_to_pose_bt_xml",
        default_value=os.path.join(
            bringup_dir, "params", "nav2z_client", "navigation_tree.xml"
        ),
        description="Full path to the behavior tree xml file to use",
    )
    
    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart", default_value="true", description="Automatically startup the nav2 stack"
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    declare_use_tags_cmd = DeclareLaunchArgument(
        "use_tags",
        default_value="False",
        description="Whether to use apriltags",
    ) 
    #...............................................................................

    
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "use_robot_state_pub",
        default_value="True",
        description="Whether to start the robot state publisher",
    )

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        arguments=[urdf],
        remappings=remappings,
        parameters=[{"params":params,"use_sim_time": use_sim_time}],

    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir,launch_dir, "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": "",
            "use_namespace": "False",
            "rviz_config": rviz_config_file,
        }.items(),
    )

    gazebo_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir,launch_dir,"gazebo_launch.py")),
        launch_arguments={"show_gz_lidar": show_gz_lidar}.items(),
    )
    
    smoothner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(smoothner_dir, 'launch','velocity_smoother-launch.py')),
        launch_arguments={
            "namespace": "",
            "use_namespace": "False",
        }.items(),
    )
    
    cmd_pid = Node(
        package='robot1',
        executable='cmd_pid.py',
        name='cmd_pid',
        output='screen',
    )
    
    tags_group = GroupAction(
        [
           IncludeLaunchDescription( 
                PythonLaunchDescriptionSource(os.path.join(apriltag_ros_dir,'launch',
                                                       'tag_realsense.launch.py')),
                condition=IfCondition(use_tags),
                launch_arguments={'namespace': namespace,}.items(),
            ),

            Node(
                package='robot1',
                executable='insert_data.py',
                name='insert_data',
                output='screen',
                condition=IfCondition(use_tags),
            )
        ]
    )

    localization = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav,'launch',
                                                       'localization_launch.py')),
                condition=IfCondition(PythonExpression(['not ', slam])),
                launch_arguments={'image_topic': image_topic,
                                  'camera_name': camera_name,
                                  'map': map,

                }.items()
    )

    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, "online_sync_launch.py")),
                condition=IfCondition(slam),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    #"resolution":0.01
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav, 'launch','navigation_launch.py')),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "default_nav_to_pose_bt_xml": default_nav_to_pose_bt_xml,
                    "use_lifecycle_mgr": "True",
                    "map_subscribe_transient_local": "False",
                }.items(),
            ),
        ]
    )

    imagem41 = Node(
        package='robot1',
        executable='imagem41.py',
        name='imagem41',
        output='screen',
    )
    
    transforma = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ["0", "0", "0", "0", "0", "0", "map", "odom"],
        condition=IfCondition(PythonExpression(["not ", slam]))
    )
     
    mapeamento_group = GroupAction(
        [
            Node(
                package='robot1',
                executable='mapeamento.py',
                name='mapeamento',
                output='screen',
                condition=IfCondition(PythonExpression(["not ", use_tags])),
            ),

            Node(
                package='robot1',
                executable='machines.py',
                name='machines',
                output='screen',
                condition=IfCondition(PythonExpression(["not ", use_tags])),
            ),
        ]
    )
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'robot1'],
                        output='screen')

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_trajectory_controller'],
        output='screen'
    )

    xtermprefix = "xterm -xrm 'XTerm*scrollBar:  true' -xrm 'xterm*rightScrollBar: true' -hold -geometry 1000x600 -sl 10000 -e"
    #...............................................................................
    ld = LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        ),])
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_show_gz_lidar)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)                            
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)

    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_tags_cmd)
    
    ld.add_action(gazebo_simulator)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    
    #ld.add_action(imagem41)
    ld.add_action(bringup_cmd_group)
    ld.add_action(cmd_pid)
    ld.add_action(localization)
    ld.add_action(transforma)
    #ld.add_action(mapeamento_group)
    ld.add_action(smoothner)
    #ld.add_action(tags_group)
    return ld

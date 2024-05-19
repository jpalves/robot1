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
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    rviz_config_file    = LaunchConfiguration("rviz_config_file")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")
    use_tags = LaunchConfiguration("use_tags")
    #...............................................................................
    nav = get_package_share_directory("nav2_bringup")
    
    image_topic = "color/image_raw"
    camera_name = "color"

    urdf = os.path.join(bringup_dir, "urdf", "amr_base.urdf")
    doc = xacro.parse(open(urdf))
    xacro.process_doc(doc)
    
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    world = os.path.join(
        get_package_share_directory('robot1'),
        'worlds',
        'hospital.world'
    )
    
    declare_namespace_cmd = DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace")
    
    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="True",
        description="Whether to apply a namespace to the navigation stack",
    )
    
    declare_show_gz_lidar = DeclareLaunchArgument(
        "show_gz_lidar",
        default_value="True",
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
        default_value=os.path.join(bringup_dir, "maps", "teste2.yaml"),
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
            bringup_dir, "params", "navigate_w_basic_complete_coverage.xml"
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
        parameters=[{"robot_description": doc.toxml(),"use_sim_time": use_sim_time}],

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

    cmd_pid = Node(
        package='robot1',
        executable='cmd_pid.py',
        name='cmd_pid',
        output='screen',
    )
    
    velos = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen', 
    )
    
    transforma = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ["0", "0", "0", "0", "0", "0", "map", "odom"],
        condition=IfCondition(PythonExpression(["not ", slam])),    
    )
    
    tags = Node(package='deeptag_ros', executable='deep_tag.py', output='screen')

    localization = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav,'launch',
                                                       'localization_launch.py')),
                condition=IfCondition(PythonExpression(['not ', slam])),
                launch_arguments={'image_topic': image_topic,
                                  'camera_name': camera_name,
                                  'map': map,
     
                }.items()
    )
    
    robot_localization_node = Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         output='screen',
         parameters=[os.path.join(bringup_dir, 'config/ekf.yaml'), {'use_sim_time': use_sim_time}]
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
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch','navigation_launch.py')),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "default_nav_to_pose_bt_xml": default_nav_to_pose_bt_xml,
                    "use_lifecycle_mgr": "False",
                    "map_subscribe_transient_local": "False",
                }.items(),
            ),
        ]
    )

    imagem = Node(
        package='robot1',
        executable='imagem42.py',
        name='imagem41',
        output='screen',
    )
    
    #...............................................................................
    ld = LaunchDescription()
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
    
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(robot_localization_node)
    ld.add_action(imagem)
    ld.add_action(transforma)
    ld.add_action(bringup_cmd_group)
    ld.add_action(velos)
    ld.add_action(cmd_pid)
    ld.add_action(localization)
    ld.add_action(tags)
    
    return ld

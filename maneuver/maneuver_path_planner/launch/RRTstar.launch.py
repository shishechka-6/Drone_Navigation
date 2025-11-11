import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from moveit_configs_utils import MoveItConfigsBuilder



def generate_launch_description():
    global_planner_param = os.path.join(
        get_package_share_directory('maneuver_path_planner'),
        'config',
        'path_planer.yaml'
    )

    local_planner_param = os.path.join(
        get_package_share_directory('maneuver_path_planner'),
        'config',
        'local_planer_params.yaml'
    )

    hybrid_planner_param = os.path.join(
        get_package_share_directory('maneuver_path_planner'),
        'config',
        'hybrid_planner_params.yaml'
    )

    urdf_path = os.path.join(
        get_package_share_directory('maneuver_bringup'),
        'urdf', 
        'drone.urdf'
    )
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    robot_description = {'robot_description': robot_desc}
    
     # SRDF — замените на путь к вашему SRDF, сгенерированному через MoveIt Setup Assistant
    srdf_path = os.path.join(
        get_package_share_directory('drone_moveit_config'),  # пакет, где хранится SRDF
        'config',
        'drone.srdf'
    )
    with open(srdf_path, 'r') as infp:
        robot_desc_semantic = infp.read()
    robot_description_semantic = {'robot_description_semantic': robot_desc_semantic}
    
    # Здесь добавляем kinematics.yaml
    kinematics_path = os.path.join(
        get_package_share_directory('drone_moveit_config'),
        'config',
        'kinematics.yaml'
    )
    # Загружаем как параметр
    kinematics_parameters = {'kinematics_yaml': kinematics_path}
    rviz_config_path = os.path.join(
        get_package_share_directory('maneuver_path_planner'),
        'rviz',
        'moveit_config.rviz'
    )


    # OMPL planner pipeline
    # ompl_params = os.path.join(
    # get_package_share_directory('maneuver_path_planner'),
    # 'config',
    # 'ompl_planning.yaml'
    # )
    # ompl_planning_pipeline_config = {'ompl_planning_pipeline_config': ompl_params}


    controllers_path = os.path.join(
        get_package_share_directory('drone_moveit_config'))
    controllers_yaml = os.path.join(controllers_path, 'config', 'moveit_controllers.yaml')


    container = ComposableNodeContainer(
            name="hybrid_planning_container",
            namespace="/",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="moveit_hybrid_planning",
                    plugin="moveit::hybrid_planning::GlobalPlannerComponent",
                    name="global_planner",
                    parameters=[
                        global_planner_param,
                        robot_description,
                        robot_description_semantic,
                        kinematics_parameters,
                        controllers_yaml,
                        # ompl_planning_pipeline_config,
                    ],
                ),
                ComposableNode(
                    package="moveit_hybrid_planning",
                    plugin="moveit::hybrid_planning::LocalPlannerComponent",
                    name="local_planner",
                    parameters=[
                        local_planner_param,
                        robot_description,
                        robot_description_semantic,
                        kinematics_parameters,
                        controllers_yaml,
                    ],
                ),
                ComposableNode(
                    package="moveit_hybrid_planning",
                    plugin="moveit::hybrid_planning::HybridPlanningManager",
                    name="hybrid_planning_manager",
                    parameters=[hybrid_planner_param],
                ),
            ],
            output="screen",
    )

    

    return LaunchDescription([
        # Node(
        #     package='moveit_simple_controller_manager',
        #     executable='ros2_control_node',
        #     parameters=[controllers_yaml],
        #     output='screen'
        # ),


        container,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        
        Node(
	    package='tf2_ros',
	    executable='static_transform_publisher',
	    arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
	    name='static_tf_map_to_base'
	    ),

        
        Node(
	    package="rviz2",
	    executable="rviz2",
	    name="rviz2",
	    output="screen",
	    arguments=["-d", rviz_config_path],
	    parameters=[robot_description, robot_description_semantic]
	    ),
	
	    Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'frame_id': 'map',
                'resolution': 0.05  # Можно изменить под ваши нужды
            }]
  )
        


    ])

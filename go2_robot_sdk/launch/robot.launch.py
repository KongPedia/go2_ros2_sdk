# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

import os
from typing import List
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument,GroupAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml


class Go2LaunchConfig:
    """Configuration container for Go2 robot launch parameters"""
    
    def __init__(self):
        # Environment variables
        self.robot_token = os.getenv('ROBOT_TOKEN', '')
        self.robot_ip = os.getenv('ROBOT_IP', '')
        self.map_name = os.getenv('MAP_NAME', '3d_map')
        self.save_map = os.getenv('MAP_SAVE', 'true')
        self.conn_type = os.getenv('CONN_TYPE', 'webrtc')
        self.num_robots = 2
        self.robot_ip_list = self._parse_ip_list(self.robot_ip)

        # Derived configurations
        self.conn_mode = self._determine_connection_mode()
        self.rviz_config = self._get_rviz_config()
        self.urdf_file = self._get_urdf_file()
        
        # Package paths
        self.package_dir = get_package_share_directory('go2_robot_sdk')
        self.config_paths = self._get_config_paths()
        
        print(f"� Go2 Launch Configuration:")
        print(f"   Robot IPs: {self.robot_ip_list}")
        print(f"   Connection: {self.conn_type} ({self.conn_mode})")
        print(f"   URDF: {self.urdf_file}")
    
    def _parse_ip_list(self, robot_ip: str) -> List[str]:
        """Parse robot IP addresses from environment variable"""
        return robot_ip.replace(" ", "").split(",") if robot_ip else []
    
    def _determine_connection_mode(self) -> str:
        """Determine connection mode based on IP list and connection type"""
        return "single" if len(self.robot_ip_list) == 1 and self.conn_type != "cyclonedds" else "multi"
    
    def _get_rviz_config(self) -> str:
        """Get appropriate RViz configuration file"""
        if self.conn_type == 'cyclonedds':
            return "cyclonedds_config.rviz"
        elif self.conn_mode == 'single':
            return "single_robot_conf.rviz"
        else:
            return "multi_robot_conf.rviz"
    
    def _get_urdf_file(self) -> str:
        """Get appropriate URDF file"""
        return 'go2.urdf'
    
    def _get_config_paths(self) -> dict:
        """Get all configuration file paths"""
        all_robot_configs = []
        for i in range(len(self.robot_ip_list)):
            
            config = {
                'joystick': os.path.join(self.package_dir, 'config', f'joystick{i}.yaml'),
                'twist_mux': os.path.join(self.package_dir, 'config', f'twist_mux{i}.yaml'),
                'slam': os.path.join(self.package_dir, 'config', f'mapper_params_online_async{i}.yaml'),
                'nav2': os.path.join(self.package_dir, 'config', f'nav2_params{i}.yaml'),
                'rviz': os.path.join(self.package_dir, 'config', self.rviz_config),
                'urdf': os.path.join(self.package_dir, 'urdf', self.urdf_file),
                'bt_xml': os.path.join(
                    self.package_dir,
                    'config',
                    'navigate_through_poses_w_replanning_and_recovery.xml',
                ),
            }
            all_robot_configs.append(config)
        
        return all_robot_configs[0] if self.conn_mode == 'single' else all_robot_configs



class Go2NodeFactory:
    """Factory for creating Go2 robot nodes"""
    
    def __init__(self, config: Go2LaunchConfig):
        self.config = config
    
    def create_launch_arguments(self) -> List[DeclareLaunchArgument]:
        """Create all launch arguments"""
        return [
            DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock if true'),
            DeclareLaunchArgument('rviz2', default_value='true', description='Launch RViz2'),
            DeclareLaunchArgument('nav2', default_value='true', description='Launch Nav2'),
            DeclareLaunchArgument('map', default_value='', description='Full path to map file to load (for localization)'),
            DeclareLaunchArgument('foxglove', default_value='true', description='Launch Foxglove Bridge'),
            DeclareLaunchArgument('joystick', default_value='true', description='Launch joystick'),
            DeclareLaunchArgument('teleop', default_value='true', description='Launch teleoperation'),
            DeclareLaunchArgument('obstacle_avoidance', default_value='false', description='Enable obstacle avoidance'),
            DeclareLaunchArgument('log_level', default_value='warn', description='ROS 2 log level (debug|info|warn|error|fatal)'),
            DeclareLaunchArgument('lidar_publish_rate', default_value='5.0', description='LiDAR publish rate (Hz)'),
            DeclareLaunchArgument('lidar_downsample_step', default_value='4', description='LiDAR downsample step'),
            DeclareLaunchArgument('lidar_max_points', default_value='25000', description='LiDAR max points'),
            DeclareLaunchArgument('lidar_deduplicate', default_value='false', description='Deduplicate LiDAR points'),
            DeclareLaunchArgument('use_cpp_lidar_accel', default_value='true', description='Enable C++ LiDAR acceleration (pybind11)'),
            DeclareLaunchArgument('lidar_intensity_threshold', default_value='0.0', description='LiDAR intensity threshold'),
        ]
    
    def create_robot_state_nodes(self) -> List[Node]:
        """Create robot state publisher nodes"""
        nodes = []
        log_level = LaunchConfiguration('log_level')
        
        if self.config.conn_mode == 'single':
            # Single robot configuration
            robot_desc = self._load_urdf_content(self.config.config_paths['urdf'])
            
            nodes.extend([
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='go2_robot_state_publisher',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'robot_description': robot_desc
                    }],
                    arguments=[
                        self.config.config_paths['urdf'],
                        '--ros-args',
                        '--log-level',
                        log_level,
                    ]
                ),
                self._create_pointcloud_to_laserscan_node()
            ])
        else:
            # Multi-robot configuration
            urdf_path = self.config.config_paths[0]['urdf']
            base_urdf = self._load_urdf_content(self.config.config_paths[0]['urdf'])
            use_sim_time = LaunchConfiguration('use_sim_time')
            for i, _ in enumerate(self.config.robot_ip_list):
                robot_desc = base_urdf
                
                nodes.extend([
                    Node(
                        package='robot_state_publisher',
                        executable='robot_state_publisher',
                        name='go2_robot_state_publisher',
                        output='screen',
                        namespace=f"robot{i}",
                        remappings=[
                            ('/tf', 'tf'),
                            ('/tf_static', 'tf_static')
                        ],
                        parameters=[{
                            'use_sim_time': use_sim_time,
                            'robot_description': robot_desc
                        }],
                        arguments=[
                            urdf_path,
                            '--ros-args',
                            '--log-level',
                            log_level,
                        ]
                    ),
                    # self._create_pointcloud_to_laserscan_node(f"robot{i}")
                ])
        
        return nodes
    
    def _load_urdf_content(self, urdf_path: str) -> str:
        """Load URDF file content"""
        with open(urdf_path, 'r') as file:
            return file.read()
    
    def _create_pointcloud_to_laserscan_node(self, namespace: str = None, namespace_temp: str = None) -> Node:
        """Create pointcloud to laserscan conversion node"""
        log_level = LaunchConfiguration('log_level')
        use_sim_time = LaunchConfiguration('use_sim_time')
        if namespace:
            # Multi-robot setup
            return Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name=f'{namespace}_pointcloud_to_laserscan',
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[
                    ('cloud_in', f'{namespace}/point_cloud2'),
                    ('scan', f'{namespace}/scan'),
                ],
                parameters=[{
                    'target_frame': 'base_link',
                    'transform_tolerance': 0.2,
                    # 'min_height': -2.0,      
                    # 'max_height': 2.0,
                    'use_sim_time': use_sim_time,   
                }],
                output='screen',
            )
        else:
            # Single robot setup
            return Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name='go2_pointcloud_to_laserscan',
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[
                    ('cloud_in', 'point_cloud2'),
                    ('scan', 'scan'),
                ],
                parameters=[{
                    'target_frame': 'base_link',
                    'transform_tolerance': 0.2,
                    'max_height': 0.5,
                }],
                output='screen',
            )
    
    def create_core_nodes(self) -> List[Node]:
        """Create core Go2 robot nodes"""
        log_level = LaunchConfiguration('log_level')
        lidar_publish_rate = ParameterValue(LaunchConfiguration('lidar_publish_rate'), value_type=float)
        lidar_downsample_step = ParameterValue(LaunchConfiguration('lidar_downsample_step'), value_type=int)
        lidar_max_points = ParameterValue(LaunchConfiguration('lidar_max_points'), value_type=int)
        lidar_deduplicate = ParameterValue(LaunchConfiguration('lidar_deduplicate'), value_type=bool)
        use_cpp_lidar_accel = ParameterValue(LaunchConfiguration('use_cpp_lidar_accel'), value_type=bool)
        lidar_intensity_threshold = ParameterValue(
            LaunchConfiguration('lidar_intensity_threshold'), value_type=float
        )
        obstacle_avoidance = ParameterValue(LaunchConfiguration('obstacle_avoidance'), value_type=bool)
        use_sim_time = LaunchConfiguration('use_sim_time')
        
        return [
            # Main robot driver (clean architecture)
            Node(
                package='go2_robot_sdk',
                executable='go2_driver_node',
                name='go2_driver_node',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robot_ip': self.config.robot_ip,
                    'token': self.config.robot_token,
                    'conn_type': self.config.conn_type,
                    'obstacle_avoidance': obstacle_avoidance,
                    'lidar_publish_rate': lidar_publish_rate,
                    'lidar_downsample_step': lidar_downsample_step,
                    'lidar_max_points': lidar_max_points,
                    'lidar_deduplicate': lidar_deduplicate,
                    'use_cpp_lidar_accel': use_cpp_lidar_accel,
                    'lidar_intensity_threshold': lidar_intensity_threshold,
                }],
            ),
        ]
    
    def create_teleop_nodes(self) -> List[Node]:
        """Create teleoperation and joystick nodes"""
        use_sim_time = LaunchConfiguration('use_sim_time', default='false')
        with_joystick = LaunchConfiguration('joystick', default='true')
        with_teleop = LaunchConfiguration('teleop', default='true')
        log_level = LaunchConfiguration('log_level')
        nodes = [] # 반환할 노드들을 담을 빈 리스트 생성
        if self.config.conn_mode == 'single':
            return [
                # Joystick node
                Node(
                    package='joy',
                    executable='joy_node',
                    condition=IfCondition(with_joystick),
                    arguments=['--ros-args', '--log-level', log_level],
                    parameters=[self.config.config_paths['joystick']]
                ),
                # Teleop twist joy node
                Node(
                    package='teleop_twist_joy',
                    executable='teleop_node',
                    name='go2_teleop_node',
                    condition=IfCondition(with_joystick),
                    arguments=['--ros-args', '--log-level', log_level],
                    parameters=[self.config.config_paths['twist_mux']],
                ),
                # Twist multiplexer
                Node(
                    package='twist_mux',
                    executable='twist_mux',
                    output='screen',
                    condition=IfCondition(with_teleop),
                    arguments=['--ros-args', '--log-level', log_level],
                    parameters=[
                        {'use_sim_time': use_sim_time},
                        self.config.config_paths['twist_mux']
                    ],
                ),
            ]
        else:
            # 멀티 모드: config_paths가 리스트임
            for i in range(len(self.config.robot_ip_list)):
                namespace = f'robot{i}'
                config = self.config.config_paths[i] # [수정] 인덱스로 접근하여 해당 로봇의 config dict 가져오기

                nodes.extend([
                    # 1. Joystick node
                    Node(
                        package='joy',
                        executable='joy_node',
                        namespace=namespace,
                        condition=IfCondition(with_joystick),
                        arguments=['--ros-args', '--log-level', log_level],
                        parameters=[config['joystick']] # [수정] config['key'] 사용
                    ),
                    # 2. Teleop twist joy node
                    Node(
                        package='teleop_twist_joy',
                        executable='teleop_node',
                        name='go2_teleop_node',
                        namespace=namespace,
                        condition=IfCondition(with_joystick),
                        arguments=['--ros-args', '--log-level', log_level],
                        parameters=[config['twist_mux']], # [수정] config['key'] 사용
                    ),
                    # 3. Twist multiplexer
                    Node(
                        package='twist_mux',
                        executable='twist_mux',
                        name='twist_mux',
                        namespace=namespace,
                        output='screen',
                        condition=IfCondition(with_teleop),
                        arguments=['--ros-args', '--log-level', log_level],
                        parameters=[
                            {'use_sim_time': use_sim_time},
                            config['twist_mux'] # [수정] config['key'] 사용
                        ],
                    ),
                ])
        return nodes
    
    def create_visualization_nodes(self) -> List[IncludeLaunchDescription]: # 반환 타입 변경
        """Carter 방식으로 네임스페이스가 적용된 RViz 실행"""
        with_rviz2 = LaunchConfiguration('rviz2', default='true')
        use_sim_time = LaunchConfiguration('use_sim_time', default='false')
        # nav2_bringup의 런치 디렉토리 경로 획득
        nav2_bringup_launch_dir = os.path.join(
            get_package_share_directory("nav2_bringup"), "launch"
        )
        
        entities = []
        
        if self.config.conn_mode == 'single':
            # 싱글 모드 (기존과 동일하거나 인자만 조정)
            entities.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, "rviz_launch.py")),
                    launch_arguments={
                        "namespace": "",
                        "use_namespace": "False",
                        "rviz_config": self.config.config_paths['rviz'],
                    }.items(),
                )
            )
        else:
            # 멀티 모드: 로봇별로 네임스페이스를 할당하여 RViz 실행 [1]
            for i in range(len(self.config.robot_ip_list)):
                robot_name = f'robot{i}'
                config = self.config.config_paths[i]
                
                entities.append(
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, "rviz_launch.py")),
                        condition=IfCondition(with_rviz2),
                        launch_arguments={
                            'use_sim_time': use_sim_time,
                            "namespace": robot_name,      # 각 로봇 이름 (robot0, robot1 등) [1]
                            "use_namespace": "True",      # 네임스페이스 사용 활성화 [1]
                            "rviz_config": config['rviz'], # 네임스페이스용 RViz 설정 파일 경로 [1]
                        }.items(),
                    )
                )
                
        return entities
    
    def create_include_launches(self) -> List[IncludeLaunchDescription]:
        """Create included launch descriptions"""
        use_sim_time = LaunchConfiguration('use_sim_time', default='false')
        with_foxglove = LaunchConfiguration('foxglove', default='true')
        with_nav2 = LaunchConfiguration('nav2', default='true')
        map_file = LaunchConfiguration('map')
        
        has_map = PythonExpression(["'", map_file, "' != ''"])

        foxglove_launch = os.path.join(
            get_package_share_directory('foxglove_bridge'),
            'launch', 'foxglove_bridge_launch.xml'
        )
        
        launch_entities = [] # 반환할 런치들을 담을 리스트

        # [수정] Foxglove는 싱글/멀티 관계없이 한 번만 실행 (포트 충돌 방지)
        launch_entities.append(
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(foxglove_launch),
                condition=IfCondition(with_foxglove),
            )
        )

        if self.config.conn_mode == 'single':
            config = self.config.config_paths # 딕셔너리

            rewritten_nav2_params = RewrittenYaml(
                source_file=config['nav2'],
                root_key=None,
                param_rewrites={
                    'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml': config['bt_xml'],
                },
                convert_types=True,
            )

            launch_entities.extend([
                # SLAM Toolbox
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('slam_toolbox'),
                                    'launch', 'online_async_launch.py')
                    ]),
                    condition=UnlessCondition(has_map),
                    launch_arguments={
                        'slam_params_file': config['slam'],
                        'use_sim_time': use_sim_time,
                    }.items(),
                ),
                # Nav2 (Localization)
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('nav2_bringup'),
                                    'launch', 'localization_launch.py')
                    ]),
                    condition=IfCondition(has_map),
                    launch_arguments={
                        'map': map_file,
                        'use_sim_time': use_sim_time,
                        'params_file': rewritten_nav2_params,
                        'autostart': 'True',
                    }.items(),
                ),
                # Nav2 (Navigation)
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('nav2_bringup'),
                                    'launch', 'navigation_launch.py')
                    ]),
                    condition=IfCondition(with_nav2),
                    launch_arguments={
                        'params_file': rewritten_nav2_params,
                        'use_sim_time': use_sim_time,
                        'map_subscribe_transient_local': 'true',
                        'autostart': 'True',
                    }.items(),
                ),
            ])
        else:
            for i in range(len(self.config.robot_ip_list)):
                robot_name = f'robot{i}'
                config = self.config.config_paths[i]

                rewritten_nav2_params = RewrittenYaml(
                    source_file=config['nav2'],
                    root_key=None,
                    param_rewrites={
                        'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml': config['bt_xml'],
                    },
                    convert_types=True,
                )

                # GroupAction으로 묶습니다. 이것이 핵심입니다!
                robot_group = GroupAction([
                    # 이 그룹 안의 모든 노드는 자동으로 robot_name 네임스페이스 아래로 들어갑니다.
                    PushRosNamespace(robot_name), 

                    # SLAM Toolbox
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([
                            os.path.join(get_package_share_directory('slam_toolbox'),
                                        'launch', 'online_async_launch.py')
                        ]),
                        condition=UnlessCondition(has_map),
                        launch_arguments={
                            # PushRosNamespace가 있으므로 여기서 namespace를 또 안 줘도 될 수 있지만
                            # 확실히 하기 위해 명시합니다.
                            'slam_params_file': config['slam'],
                            'use_sim_time': use_sim_time,
                        }.items(),
                    ),

                    # Nav2 (Localization)
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([
                            os.path.join(get_package_share_directory('nav2_bringup'),
                                        'launch', 'localization_launch.py')
                        ]),
                        condition=IfCondition(has_map),
                        launch_arguments={
                            'namespace': robot_name,
                            'use_namespace': 'True',
                            'map': map_file,
                            'use_sim_time': use_sim_time,
                            'params_file': rewritten_nav2_params,
                            'use_composition': 'False', 
                            'autostart': 'True',
                        }.items(),
                    ),

                    # Nav2 (Navigation)
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([
                            os.path.join(get_package_share_directory('nav2_bringup'),
                                        'launch', 'navigation_launch.py')
                        ]),
                        condition=IfCondition(with_nav2),
                        launch_arguments={
                            'namespace': robot_name,
                            'use_namespace': 'True',
                            'params_file': rewritten_nav2_params,
                            'use_sim_time': use_sim_time,
                            'map_subscribe_transient_local': 'true',
                            'autostart': 'True',
                        }.items(),
                    ),
                ])
                
                launch_entities.append(robot_group)
                    
        return launch_entities



def generate_launch_description():
    """Generate the launch description for Go2 robot system"""
    
    # Initialize configuration and factory
    config = Go2LaunchConfig()
    factory = Go2NodeFactory(config)
    
    # Create all components
    launch_args = factory.create_launch_arguments()
    robot_state_nodes = factory.create_robot_state_nodes()
    core_nodes = factory.create_core_nodes()
    teleop_nodes = factory.create_teleop_nodes()
    visualization_nodes = factory.create_visualization_nodes()
    include_launches = factory.create_include_launches()
    
    # Combine all elements
    launch_entities = (
        launch_args +
        robot_state_nodes +
        core_nodes +
        teleop_nodes +
        visualization_nodes +
        include_launches
    )
    
    return LaunchDescription(launch_entities)
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction



def generate_launch_description():

    package_name = 'diff_drive_robot'

    # ======================
    # Launch Config
    # ======================
    use_rviz = LaunchConfiguration('rviz')

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Run RViz'
    )
    
    # ======================
    # Robot State Publisher
    # ======================
    urdf_path = os.path.join(get_package_share_directory(package_name),'urdf','robot.urdf')

    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), 
                launch_arguments={'use_sim_time': 'false', 'xacro': urdf_path}.items()
    ) 
    # joint_state_publisher_node = Node(
    # package='joint_state_publisher',
    # executable='joint_state_publisher',
    # name='joint_state_publisher',
    # output='screen'
    # )

    # ======================
    # STM32 DRIVER NODE
    # ======================
    # stm32_driver = Node(
    # package='diff_drive_robot',
    # executable='cmdvel_uart',
    # name='cmdvel_uart',
    # output='screen'
    # )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": False},
            os.path.join(
                get_package_share_directory("diff_drive_robot"),
                "config",
                "controllers.yaml",
            ),
        ],
    )

  
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("diff_drive_robot"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )

    # joystick = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("diff_drive_robot"),
    #         "launch",
    #         "joystick_teleop.launch.py"
    #     ),
    #     launch_arguments={
    #         "use_sim_time": "False"
    #     }.items()
    # )
    # ======================
    # IMU NODE
    # ======================
    # imu_node = Node(
    #     package='diff_drive_robot',     
    #     executable='imu',
    #     name='imu',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': False}
    #     ]
    # )

    # ======================
    # EKF (robot_localization)
    # ======================
    ekf_config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'ekf.yaml'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': False}
        ]
    )
    ekf_node_delayed = TimerAction(
    period= 5.0,   # delay 3 giây
    actions=[ekf_node]
)
    # laser_driver = Node(
    #         package="rplidar_ros",
    #         executable="rplidar_composition",
    #         name="rplidar_node",
    #         parameters=[os.path.join(
    #             get_package_share_directory("diff_drive_robot"),
    #             "config",
    #             "rplidar_a1.yaml"
    #         )],
    #         output="screen"
    # )
    laser_driver =   Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
             parameters=[os.path.join(
                get_package_share_directory("diff_drive_robot"),
                "config",
                 "rplidar_a1.yaml"
            )],
        output="screen"
    )



    # ======================
    # RVIZ
    # ======================
    rviz_config = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'bot.rviz'
    )

    rviz_node = GroupAction(
        condition=IfCondition(use_rviz),
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config],
                output='screen'
            )
        ]
    )

    # ======================
    # RETURN
    # ======================
    return LaunchDescription([
    declare_rviz,
    rsp,
    # joint_state_publisher_node,  
    # stm32_driver,
    # imu_node,
    # interfaces,
    controller_manager,
    controller,
    # joystick,
    laser_driver,
    # ekf_node,
    rviz_node,
    ekf_node_delayed,
])
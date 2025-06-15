#############################################################################
import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

import launch_ros.actions
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument

# from launch_ros.parameters import declare_parameter

def generate_launch_description():
    # arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    
    points_topic = LaunchConfiguration('points_topic', default='/livox/lidar')
    odom_child_frame_id = LaunchConfiguration('odom_child_frame_id', default='livox_frame')
    # optional arguments
    use_imu = LaunchConfiguration('use_imu', default='false')
    invert_imu_acc = LaunchConfiguration('invert_imu_acc', default='false')
    invert_imu_gyro = LaunchConfiguration('invert_imu_gyro', default='false')
    use_global_localization = LaunchConfiguration('use_global_localization', default='False')
    imu_topic = LaunchConfiguration('imu_topic', default='/livox/imu')
    enable_robot_odometry_prediction = LaunchConfiguration('enable_robot_odometry_prediction', default='false')
    robot_odom_frame_id = LaunchConfiguration('robot_odom_frame_id', default='odom')
    plot_estimation_errors = LaunchConfiguration('plot_estimation_errors', default='false')

    
    # include hdl_global_localization launch file
    global_hdl_node = Node(
            package='hdl_global_localization',
            executable='hdl_global_localization_node',
            name='hdl_global_localization',
            output='screen',
            condition=IfCondition(use_global_localization)
        )
    
    # Transform
    odom_tf = Node(
        name='odom2base_link_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0', '0',
                   '0', '1', 'odom', 'base_link']
    ) 
    robot_tf = Node(
        name='robot2lidar_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.115', '0.0', '0.16', '0', '0',
                   '0', '1', 'base_link', 'livox_frame']
    )


    #globalmappath
    globalmap_pcd_path = os.path.join(get_package_share_directory('hdl_localization'), 'data', 'damhun.pcd')
    

    container = ComposableNodeContainer(
        name='container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='hdl_localization',
                plugin='hdl_localization::GlobalmapServer',
                name='GlobalmapServer',
                parameters=[
                    {'globalmap_pcd': globalmap_pcd_path},
                    {'convert_utm_to_local': True},
                    {'downsample_resolution': 0.2}]),
            ComposableNode(
                package='hdl_localization',
                plugin='hdl_localization::HdlLocalization',
                name='HdlLocalization',
                # remapping
                remappings=[('/velodyne_points', points_topic), ('/gpsimu_driver/imu_data', imu_topic)],
                parameters=[
                    {'odom_child_frame_id': odom_child_frame_id},
                    {'use_imu': use_imu},
                    {'invert_acc': invert_imu_acc},
                    {'invert_gyro': invert_imu_gyro},
                    {'cool_time_duration': 0.1},
                    {'enable_robot_odometry_prediction': enable_robot_odometry_prediction},
                    {'robot_odom_frame_id': robot_odom_frame_id},
                    # <!-- available reg_methods: NDT_OMP, NDT_CUDA_P2D, NDT_CUDA_D2D-->
                    {'reg_method': 'NDT_OMP'},
                    {'ndt_neighbor_search_method': 'DIRECT7'},
                    {'ndt_num_thread': 8},
                    {'ndt_neighbor_search_radius': 1.0},
                    {'ndt_resolution': 0.5},
                    {'downsample_resolution': 0.2},
                    {'specify_init_pose': True},
                    {'init_pos_x': 0.0},
                    {'init_pos_y': 0.0},
                    {'init_pos_z': 0.0},
                    {'init_ori_w': 1.0},
                    {'init_ori_x': 0.0},
                    {'init_ori_y': 0.0},
                    {'init_ori_z': 0.0},
                    {'use_global_localization': use_global_localization}])
        ],
        output='screen',   
    )


    return LaunchDescription([launch_ros.actions.SetParameter(name='use_sim_time', value=True), odom_tf, robot_tf, global_hdl_node, container])
    
    # return LaunchDescription([container])


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch.actions import GroupAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os.path



def generate_launch_description():
    

   
    
    


   

    fastlio_config = os.path.join(
        get_package_share_directory('fast_lio'), 'config', 'mid360.yaml')
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[fastlio_config],
        output='screen'
    )

    base_odom_update_node = Node(
        package='trunk_package',
        executable='base_odom_update',
        parameters=[],
        output='screen', 
    )

    
   
    return LaunchDescription([
        
        fast_lio_node,
        
        base_odom_update_node,
        
        
    ])
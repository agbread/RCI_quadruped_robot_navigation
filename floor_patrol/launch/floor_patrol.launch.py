from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bt = Node(
        package='floor_patrol',
        executable='bt_node_main',
        name='floor_patrol',
        output='screen',
        parameters=['config/patrol_params.yaml'],
        arguments=['--bt_xml', 'bt_trees/floor_patrol.xml']
    )

    # 필요시 nav2 bringup, map_server, lifecycle_mgr 포함 (이미 떠있다면 생략)
    # 여기서는 bt 노드만 예시로 띄움.

    return LaunchDescription([bt])

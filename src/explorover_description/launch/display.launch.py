from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', 
                 '/opt/ros/<distro>/share/urdf_tutorial/rviz/urdf.rviz'],
            output='screen')
    ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from time import sleep

robot_count = 3

launch_description = LaunchDescription()
package_name = 'swarmbot'

def generate_robots(amount):
    robots = []

    for i in range(amount):
        name = "bot"+str(i)
        x_offset = float(-i * 2.5)
        y_offset = float(-i * 2.5)
        robots.append({'name':name, 'x':x_offset, 'y': y_offset})

    return robots

def create_robot_spawn_description():

    while True:
        robot_count = input("Enter the amount of robots to spawn: ")
        try:
            robot_count = int(robot_count)
        except:
            print("! Please enter an integer value !")
            sleep(2)
            continue
        break

    print(robot_count)
    robots = generate_robots(robot_count)

    for bot in robots:

        temp_spawner = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', bot['name'],
                                   '-robot_namespace', bot['name'],
                                   '-topic', 'robot_description',
                                   '-x', str(bot['x']),
                                   '-y', str(bot['y'])],
                        output='screen')
        launch_description.add_action(temp_spawner)
        

        temp_spawner_angle = Node(package='prototype', executable='swarm',
                        parameters=[{'bot_id': bot['name'], 'bot_offset':str(bot['x'])}],
                        output='screen')
        launch_description.add_action(temp_spawner_angle)
        

def generate_launch_description():
    
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'),
        )
    )


    launch_description.add_action(start_world)

    state_publisher = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name),'launch','state_publisher.launch.py'
                )])
        )
    
    launch_description.add_action(state_publisher)

    create_robot_spawn_description()

    return launch_description
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(                                 # -- Узел запуска окна с черепахой
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(                                 # -- Узел транслятора системы координат черепашки_1 в систему tf2  
            package='time_travel',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
        DeclareLaunchArgument(                # -- Декларация целевого фреймя, по умолчанию это фрейм черепашки один 
            'target_frame', default_value='turtle1',
            description='Target frame name.'
        ),
        Node(                                # -- Узел транслятора системы координат черепашки_2 в систему tf2
            package='time_travel',
            executable='turtle_tf2_broadcaster',
            name='broadcaster2',
            parameters=[
                {'turtlename': 'turtle2'}
            ]
        ),
        Node(                               # -- Узел для прослушки трансформаций, в данном случае черепашки_1 из tf2 
            package='time_travel',
            executable='turtle_tf2_listener',
            name='listener',
            parameters=[
                {'target_frame': LaunchConfiguration('target_frame'), 'delay' : '5.0'} # Прослушка черепашки_1 с задержкой 5 секунд
            ]
        ),
    ])

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class TextToCmdVel(Node):
    def __init__(self):
        super().__init__('text_to_cmd_vel')

        # Подписка на топик с текстовыми командами
        self.subscription = self.create_subscription(
            String,
            'cmd_text',
            self.cmd_text_callback,
            10)
        
        # Публикация команд скорости в топик /turtle1/cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info('Text to CmdVel Node has been started.')

    def cmd_text_callback(self, msg):
        command = msg.data.strip().lower()
        twist = Twist()

        if command == 'move_forward':
            twist.linear.x = 1.0  # Черепаха движется вперед с 1 м/с
            twist.angular.z = 0.0
        elif command == 'move_backward':
            twist.linear.x = -1.0  # Черепаха движется назад с -1 м/с
            twist.angular.z = 0.0
        elif command == 'turn_right':
            twist.linear.x = 0.0
            twist.angular.z = -1.5  # Черепаха поворачивает направо с -1.5 радиан/с
        elif command == 'turn_left':
            twist.linear.x = 0.0
            twist.angular.z = 1.5  # Черепаха поворачивает налево с 1.5 радиан/с
        else:
            self.get_logger().warn(f'Unknown command: {command}')
            return

        # Публикуем сообщение типа Twist в топик /turtle1/cmd_vel
        self.publisher_.publish(twist)
        self.get_logger().info(f'Published command: {command}')

def main(args=None):
    rclpy.init(args=args)

    # Создаем и запускаем узел
    node = TextToCmdVel()

    # Спин узла для обработки сообщений
    rclpy.spin(node)

    # Очистка ресурсов
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

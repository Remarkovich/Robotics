import sys
from my_custom_package.srv import FullNameSumService
import rclpy
from rclpy.node import Node


class ClientNameNode(Node):

    def __init__(self):
        super().__init__('client_name')
        self.client = self.create_client(FullNameSumService, 'summ_full_name')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request = FullNameSumService.Request()

    def send_request(self, last_name, name, first_name):
        self.request.last_name = last_name
        self.request.name = name
        self.request.first_name = first_name

        future = self.client.call_async(self.request) # асинхронный вызов сервиса
        rclpy.spin_until_future_complete(self, future) #  ожидаем завершения асинхронного вызова
        return future.result()


def main():
    rclpy.init()

    client_name_node = ClientNameNode()
    response = client_name_node.send_request(sys.argv[1], sys.argv[2], sys.argv[3])

    if response:
        client_name_node.get_logger().info(
   	    'Full Name: %s' % response.full_name)
    else:
        client_name_node.get_logger().error('Ошибка при вызове сервиса')

    client_name_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()

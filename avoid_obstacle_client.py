import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class AvoidObstacleClient(Node):
    def __init__(self):
        super().__init__('avoid_obstacle_client')
        self.client = self.create_client(SetBool, '/pirobot/avoid_obstacle')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service to become available...')
        self.req = SetBool.Request()

    def send_request(self, data: bool):
        self.req.data = data
        future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    ao_client = AvoidObstacleClient()

    input_command = input("Enter 'start' to begin or 'stop' to end: ").strip().lower()
    if input_command == 'start':
        response = ao_client.send_request(True)
        print(f"Response: {response.message}")
    elif input_command == 'stop':
        response = ao_client.send_request(False)
        print(f"Response: {response.message}")
    else:
        print("Invalid command")
    
    ao_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

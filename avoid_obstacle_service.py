import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from pirobot import motor_control as mc
from pirobot import avoid_obstacle as oa
import time


class AvoidObstacleService(Node):
    def __init__(self):
        super().__init__('avoid_obstacle_service')
        self.srv = self.create_service(SetBool, '/pirobot/avoid_obstacle', self.avoid_obstacle_callback)
        self.timer = None  # Timer for autonomous motion
        self.is_running = False

    def avoid_obstacle_callback(self, request, response):
        if request.data:  # Start autonomous motion
            if not self.is_running:
                self.get_logger().info('Received request to start autonomous motion...')
                self.is_running = True
                self.timer = self.create_timer(0.1, self.motion_callback)  # Set periodic motion checks
                response.success = True
                response.message = "Robot is now avoiding obstacles."
            else:
                response.success = False
                response.message = "Robot is already running."
        else:  # Stop autonomous motion
            if self.is_running:
                self.get_logger().info('Received request to stop motion...')
                self.destroy_timer(self.timer)  # Stop the timer
                mc.stopmotors()  # Stop motors
                self.is_running = False
                response.success = True
                response.message = "Robot stopped."
            else:
                response.success = False
                response.message = "Robot is not running."
        return response

    def motion_callback(self):
        """Callback function for autonomous motion."""
        if oa.is_near_obstacle(15.0):  # Check for obstacles
            self.get_logger().info('Obstacle detected. Avoiding...')
            mc.stopmotors()
            mc.backward()
            time.sleep(0.5)
            mc.turnright()
            time.sleep(0.75)
        else:
            mc.forward()


def main(args=None):
    rclpy.init(args=args)
    ao_service = AvoidObstacleService()
    try:
        rclpy.spin(ao_service)
    except KeyboardInterrupt:
        ao_service.get_logger().info('Service interrupted. Shutting down...')
    finally:
        ao_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

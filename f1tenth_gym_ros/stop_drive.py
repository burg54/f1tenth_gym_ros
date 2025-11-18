
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class StopDrive(Node):

    def __init__(self):
        super().__init__("stop_drive")
        # must publish to drive. 
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 1)
        self.stop_timer = self.create_timer(0.1, self.check_stop_car)

    def check_stop_car(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0.0
        drive_msg.drive.steering_angle = 0.0
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StopDrive()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info(f"Car Stopped.")
    node.destroy_node()

if __name__ == "__main__":
    main()

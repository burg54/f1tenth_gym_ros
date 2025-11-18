
# import rospy
import numpy as np
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class FollowTheGap(Node):

    def __init__(self):
        super().__init__("publish_gap")
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, "/drive", 1
        )  #! must publish to drive. this is how the kill switch can work when in use. Messages must be stamped
        self.scan = self.create_subscription(
            LaserScan(),
            #"/picoScan_23460001/scan/all_segments_echo0",
            "/scan",
            self.lidar_callback,
            1,
        )
        self.scan  # prevents unused variable warning
        self.lidar_fov = np.radians(180)
        self.max_speed = 5

        self.driving_style = "FTG"  # options are FTG, RL, or student

        self.pub_drive(
            0.0, 0.0
        )  # added so the steering angle and speed always reset to 0 before driving

    def preprocess_lidar(self, ranges, window_size=15, max_value=10, avoid=50):
        """Preprocess the LiDAR scan array.
        Args:
            ranges (List[float]): A list of ranges from the LiDAR scan.
            window_size (int): The size of the window for calculating the mean.
            max_value (float): The maximum value reject anything above this.
        Returns:
            List[float]: The preprocessed LiDAR scan array.
        """

        # following code not necessary for sim, because
        # sim lidar does not have noise
        processed_data = ranges

        """
        if len(ranges) > 0:
            # Initialize result list with the same length as ranges
            processed_data = [0] * len(ranges)
            minima = min(ranges)
            loc_minima = ranges.index(minima)
            # Apply moving average with window_size
            for i in range(len(ranges)):
                if ranges[i] > max_value:
                    ranges[i] = 0
                if abs(loc_minima - i) <= (avoid / 2):
                    ranges[i] = 0
                window = ranges[max(0, i - window_size + 1) : i + 1]
                avg = sum(window) / len(window) if window else 0
                processed_data[i] = avg
        else:
            return 1
        """

        return processed_data

    def find_max_gap(self, free_space_ranges):
        """Return the start index & end index of the max gap in free_space_ranges"""
        max_gap_start = 0
        max_gap_end = 0
        max_gap_length = 0
        maxima = 0
        maxima = max(free_space_ranges)

        gap_start = None
        gap_end = None
        for i in range(len(free_space_ranges)):
            if free_space_ranges[i] > min(
                max(1.5, maxima / 3), 4
            ):  #! Make this adaptive at some point 1/2 the max length perhaps ## this is a bad idea as it may drive into wall this number becomes to small so use max
                if gap_start is None:
                    gap_start = i
                else:
                    gap_end = i
            else:
                if gap_start is not None and gap_end is not None:
                    gap_length = gap_end - gap_start
                    if gap_length > max_gap_length:
                        max_gap_length = gap_length
                        max_gap_start = gap_start
                        max_gap_end = gap_end
                    gap_start = None
                    gap_end = None
        if gap_start is not None and gap_end is not None:
            gap_length = gap_end - gap_start
            if gap_length > max_gap_length:
                max_gap_length = gap_length
                max_gap_start = gap_start
                max_gap_end = gap_end


        return max_gap_start, max_gap_end

    def lidar_callback(self, data):
        """Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message"""
        # Convert ranges to a list
        ranges = list(data.ranges)

        if self.driving_style == "FTG":
            self.FTG_drive(ranges, data.angle_increment, data.angle_min)
        elif self.driving_style == "RL":
            pass  #! add later
        elif self.driving_style == "student":
            pass  #! add later

    def FTG_drive(self, ranges, increment, min):
        # added to handle 270 degree scan #! changed the fov of the lidar unit so no longer needed
        # cut_points = int(0.5*(data.angle_max - self.lidar_fov)/data.angle_increment)
        # ranges=ranges[cut_points:-cut_points] #points are cut first so processing lidar is less computationally expensive
        # process lidar
        front_avg = sum(ranges[15:35]) / 20
        back_avg = sum(ranges[1045:1065]) / 20

        ranges = self.preprocess_lidar(
            ranges
        )  #! add this back later self.preprocess_lidar(data.ranges)
        if ranges == 1:
            self.pub_drive(0.0, 0.0)
            return
        # proc_ranges = self.preprocess_lidar(ranges)
        gap_start, gap_end = self.find_max_gap(ranges)
        best_point = (gap_start + gap_end) / 2
        steering_angle = (
            best_point
        ) * increment + min  # ((best_point+cut_points)*data.angle_increment + data.angle_min)#*(2.5-speed)*0.5
        # if steering_angle > 0: #right is negative left is positive
        #     if back_avg < 0.2:
        #         steering_angle = steering_angle/8
        # else:
        #     if front_avg < 0.2:
        #         steering_angle = steering_angle/8
        speed = (
            1.25 + (1 / (steering_angle**2 + 0.01)) / 50
        )  # 1.25+(1/(steering_angle**2+0.01))/50 #this works but want to go faster #function adds max 2  #min(1.2 + ranges[int(len(ranges)/2)]/6, 3.5) #!changing speed to be based off of the steering angle
        self.max_speed = 1.25 + (1 / (0**2 + 0.01)) / 50  # make sure same as above
        self.pub_drive(
            speed, steering_angle
        )  # publish drive message every time the lidar is published

    def pub_drive(self, speed, steering_angle):
        # publish drive messages from speed and steering angle
        #! could alter equations to use accelerations
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )  # Comented out because ackerman drive does not have a header. add back if stamped message is used
        drive_msg.drive.speed = speed  #! must be a float
        drive_msg.drive.steering_angle = steering_angle  # *(2.3-speed)
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)

    follow_the_gap = FollowTheGap()
    rclpy.spin(follow_the_gap)

    # Destroy the node once things have ended
    # not required
    #follow_the_gap.destroy_node()
    #rclpy.shutdown()


if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class EBS(Node):

    def __init__(self):
        super().__init__('ebs')
        self.xvel = 0
        self.publisher = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.subscription = self.create_subscription(LaserScan,'scan',self.listener_callback,10)
        self.subscription = self.create_subscription(Twist,'cmd_vel',self.listener_callback_speed,10)
    
    def listener_callback_speed(self, msg):
        self.xvel = msg.linear.x

    def listener_callback(self, msg):
        inc = math.degrees(msg.angle_increment)
        num_measurement = len(msg.ranges)
        scan_start_angle = math.degrees(msg.angle_min)
        scan_filter_cone_angle = 10
        total_scan_radius = -math.degrees(msg.angle_min) + math.degrees(msg.angle_max)
        scans_amount = len(msg.ranges)
        scans_per_degree = scans_amount / total_scan_radius
        middle_index = scans_amount // 2
        cone_indeces_half = int(scan_filter_cone_angle / 2 * scans_per_degree)
        filtered_ranges = msg.ranges[middle_index - cone_indeces_half:middle_index + cone_indeces_half]

        shouldBrake = False
        derivatives = [self.xvel * math.cos(inc * i - scan_filter_cone_angle // 2) for i in range(len(filtered_ranges))]

        iTTC = [(a / b if b != 0 else float('inf')) for a, b in zip(filtered_ranges, [max(-x, 0.0) for x in derivatives])] 
        
        minimum = min(iTTC)
        if minimum != float('inf'):
            print(f"\r{minimum}", end="", flush=True)

        for value in iTTC:
            if value < 0.5:
                shouldBrake = True
                break
        
        brakemsg = AckermannDriveStamped()

        if shouldBrake == True:
            self.publisher.publish(brakemsg)
        shouldBrake = False


def main(args=None):
    rclpy.init(args=args)

    ebs = EBS()

    rclpy.spin(ebs)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ebs.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

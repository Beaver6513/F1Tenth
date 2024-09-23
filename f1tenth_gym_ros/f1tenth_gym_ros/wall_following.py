import rclpy
from rclpy.node import Node
import os
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import math as m
from nav_msgs.msg import Odometry
import numpy as np

def lerp(a: float, b: float, t: float) -> float:
    return (1 - t) * a + t * b

def inv_lerp(a: float, b: float, v: float) -> float:
    return (v - a) / (b - a)

def remap(i_min: float, i_max: float, o_min: float, o_max: float, v: float) -> float:
    return lerp(o_min, o_max, inv_lerp(i_min, i_max, v))

def eerp(x, y, z, k=5) -> float:
    # Ensure z is within the bounds of 0 and 1
    z = np.clip(z, 0, 1)
    # Perform exponential interpolation
    return x + (y - x) * (np.exp(k * z) - 1) / (np.exp(k) - 1)

class WALLFOLLOWER(Node):
    def __init__(self):
        super().__init__('wallfollower')
        self.xvel = 0
        self.max_vel = 4
        self.integral_error = 0.0001
        self.prev_error = 0.0001
        self.error = 0.0
        self.prev_time = 0
        self.isfirst = True
        self.publisher = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.subscription = self.create_subscription(LaserScan,'scan',self.listener_callback,10)
        self.subscription = self.create_subscription(Odometry,'ego_racecar/odom',self.listener_callback_speed,10)
    
    def PID_Controller(self, msg):
        kp = 0.6
        ki = 2
        kd = 0.125
        p_component = self.error * kp
        crnt_time = msg.header.stamp.sec + msg.header.stamp.nanosec / (10 ** 9)
        dt = crnt_time - self.prev_time
        if dt == 0:
            dt = 0.0001

        self.integral_error += self.error * dt
        i_component = ki * self.integral_error
        d_component = (self.error - self.prev_error) / dt * kd
        return p_component + i_component + d_component

    def listener_callback_speed(self, msg):
        self.xvel = msg.twist.twist.linear.x

    def listener_callback(self, msg):
        #---------------------
        if self.isfirst:
            self.prev_time = msg.header.stamp.sec + (msg.header.stamp.nanosec - 100) / (10 ** 9)
            self.isfirst = False
        
        total_scan_radius = -m.degrees(msg.angle_min) + m.degrees(msg.angle_max)
        scans_amount = len(msg.ranges)
        scans_per_degree = scans_amount / total_scan_radius
        middle_index = scans_amount // 2
        b_angle = 50
        wall_90_index = middle_index - 90 * scans_per_degree
        wall_b_angle_index = middle_index - b_angle * scans_per_degree
        b = msg.ranges[int(wall_90_index)]
        a = msg.ranges[int(wall_b_angle_index)]
        theta_deg = 90 - b_angle
        theta = m.radians(theta_deg)
        desired_dist = 1
        #----------------------
        
        num = a * m.cos(theta) - b
        den = a * m.sin(theta)

        alpha = m.atan(num / den)

        wall_d = b * m.cos(alpha)
        l = remap(0.5, self.max_vel, 0, 2, self.xvel)
        wall_d_n = wall_d + l * m.sin(alpha) 
        self.error = desired_dist - wall_d_n
        steer = self.PID_Controller(msg)
        self.prev_error = self.error
        speed = eerp(self.max_vel, 0.5, abs(steer))

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steer
        drive_msg.drive.speed = float(speed)
        self.publisher.publish(drive_msg)

        crnt_time = msg.header.stamp.sec + msg.header.stamp.nanosec / (10 ** 9)
        
        #---DEBUG----------------------------------------
        os.system('cls' if os.name == 'nt' else 'clear')
        print(f'Wall Distance {wall_d}', flush=True)
        print(f'Wall Distance n {wall_d_n}', flush=True)
        print(f'Wall Distance_a {a}', flush=True)
        print(f'Wall Distance_b {b}', flush=True)
        print(f'Alpha deg {m.degrees(alpha)}', flush=True)
        print(f'error {self.error}', flush=True)
        print(f'Integral error {self.integral_error}', flush=True)
        print(f'steer {steer}', flush=True)
        print(f'speed {speed}', flush=True)
        print(f'dt {crnt_time - self.prev_time}', flush=True)
        print(f'l {l}', flush=True)
        print(f'xvel {self.xvel}', flush=True)

        self.prev_time = crnt_time





def main(args=None):
    rclpy.init(args=args)

    wall_following = WALLFOLLOWER()

    rclpy.spin(wall_following)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_following.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

from numpy.core.fromnumeric import clip
import rclpy
from rclpy.node import Node
import os
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import math as m
import numpy as np


def lerp(a: float, b: float, t: float) -> float:
    return (1 - t) * a + t * b


def inv_lerp(a: float, b: float, v: float) -> float:
    return (v - a) / (b - a)


def remap(i_min: float, i_max: float, o_min: float, o_max: float, v: float) -> float:
    return lerp(o_min, o_max, inv_lerp(i_min, i_max, v))


def eerp(x, x_min, x_max, y_min, y_max, k=5) -> float:
    # Perform exponential interpolation
    normalized_x = (x - x_min) / (x_max - x_min)
    y_normalized = (np.exp(k * normalized_x) - 1) / (np.exp(k) - 1)
    return y_min + y_normalized * (y_max - y_min)


def ieerp(x, x_min, x_max, y_min, y_max, k=5) -> float:
    # Perform inverted exponential interpolation
    normalized_x = (x - x_min) / (x_max - x_min)
    y_normalized = 1 - (np.exp(k * (1 - normalized_x)) - 1) / (np.exp(k) - 1)
    return y_min + y_normalized * (y_max - y_min)


def dbg_print(msg: str, var):
    print(f"{msg}{str(var)}", flush=True)


class Gap:
    def __init__(self, max_depth=0, start=0, end=0, max_i=0):
        self.max_depth = max_depth
        self.start = start
        self.end = end
        self.max_i = max_i


class DEGF(Node):
    def __init__(self):
        super().__init__("degf")
        self.cone_radius = 180
        self.disparity_threshold = 2
        self.isfirst = True
        self.min_gap_depth = 5
        self.car_w = 0.5
        self.max_steer = 50.0
        self.xvel = 0
        self.max_vel = 8.0
        self.clamp = 10
        self.gap_list = []
        self.publisher = self.create_publisher(AckermannDriveStamped, "drive", 10)
        self.subscription = self.create_subscription(
            LaserScan, "scan", self.listener_callback, 10
        )
        self.subscription = self.create_subscription(
            Odometry, "ego_racecar/odom", self.listener_callback_speed, 10
        )

    def listener_callback_speed(self, msg):
        self.xvel = msg.twist.twist.linear.x

    def listener_callback(self, msg):
        os.system("cls" if os.name == "nt" else "clear")
        total_scan_radius = -m.degrees(msg.angle_min) + m.degrees(msg.angle_max)
        scans_amount = len(msg.ranges)
        s_per_deg = scans_amount / total_scan_radius
        middle_index = scans_amount // 2
        cone_limit_index_amount = int(self.cone_radius / 2 * s_per_deg)
        scan = msg.ranges
        fscan = scan[
            middle_index
            - cone_limit_index_amount : middle_index
            + cone_limit_index_amount
        ]
        fs_len = len(fscan)

        # ---Scan filtering with disparity extension----------
        if 1:
            if 1:
                i = 0
                while i < fs_len - 1:
                    if (
                        fscan[i] + self.disparity_threshold < fscan[i + 1]
                        and fscan[i] != 0.0
                    ):
                        alpha = m.degrees(m.atan(self.car_w / 2 / fscan[i]))
                        index = max(int(alpha * s_per_deg), 1)
                        for x in range(min(index, fs_len - i)):
                            fscan[i + x] = 0.0  # fscan[i]
                        i += index + 1  # Manually increment i by index
                    else:
                        i += 1  # Standard increment when the condition is not met

            if 1:
                i = fs_len - 1
                while i > 1:
                    if (
                        fscan[i] + self.disparity_threshold < fscan[i - 1]
                        and fscan[i] != 0.0
                    ):
                        alpha = m.degrees(m.atan(self.car_w / 2 / fscan[i]))
                        index = max(int(alpha * s_per_deg), 1)
                        for x in range(min(index, i)):
                            fscan[i - x] = 0.0  # fscan[i]
                        i -= index + 1  # Manually decrement i by index
                    else:
                        i -= 1  # Standard decrement when the condition is not met

        # ---Gap search algorithm------------
        i = 0
        if 0:
            while i < fs_len:
                if fscan[i] > 0.0:  # self.min_gap_depth:
                    gap_s = i
                    gap_e = gap_s
                    max_d = fscan[i]
                    while i < fs_len and fscan[i] > 0.0:  # self.min_gap_depth:
                        gap_e += 1
                        i += 1
                    max_d = max(fscan[gap_s:gap_e])
                    max_i = fscan.index(max_d)
                    if gap_e - gap_s >= 10:
                        self.gap_list.append(Gap(max_d, gap_s, gap_e, max_i))
                else:
                    i += 1

        if 1:
            while i < fs_len:
                if fscan[i] > self.min_gap_depth:
                    gap_s = i
                    gap_e = gap_s
                    max_d = fscan[i]
                    while i < fs_len and fscan[i] > self.min_gap_depth:
                        gap_e += 1
                        i += 1
                    max_d = max(fscan[gap_s:gap_e])
                    max_i = fscan.index(max_d)
                    if gap_e - gap_s >= 6:
                        self.gap_list.append(Gap(max_d, gap_s, gap_e, max_i))
                else:
                    i += 1

        # ---Cornering check------------
        choosen_index = fs_len // 2
        if len(self.gap_list) >= 1:
            choosen_gap = max(self.gap_list, key=lambda x: x.max_depth)
            a = 0
            if a == 1:
                choosen_index = choosen_gap.max_i
            else:
                choosen_index = (choosen_gap.start + choosen_gap.end) // 2
        else:
            choosen_index = fscan.index(max(fscan))

        steer = (1 / s_per_deg) * (choosen_index - fs_len // 2)

        is_at_wall = 1.0
        if 1:
            if any(
                l < self.car_w / 2 for l in scan[0 : middle_index - int(90 * s_per_deg)]
            ):
                is_at_wall = 0.0
            if any(
                l < self.car_w / 2
                for l in scan[middle_index + int(90 * s_per_deg) : len(scan)]
            ):
                is_at_wall = 0.0

        if 1:
            for x in range(fs_len):
                if fscan[x] > self.clamp:
                    fscan[x] = self.clamp

        # ---Drive Message---------
        drive_msg = AckermannDriveStamped()

        # ---Steer clamp------------
        if steer > self.max_steer:
            steer = self.max_steer
        if steer < -self.max_steer:
            steer = -self.max_steer

        prevision = 2.0
        speed = ieerp(
            max(0.0, fscan[choosen_index] - prevision),
            0,
            self.clamp,
            2,
            self.max_vel,
            k=2,
        )

        drive_msg.drive.steering_angle = m.radians(steer) * is_at_wall

        steer_speed_coefficient = eerp(abs(steer), 0, self.max_steer, 1, 0.2, 5)

        drive_msg.drive.speed = speed * steer_speed_coefficient
        self.publisher.publish(drive_msg)

        # ---DEBUG----------------------------------------
        dbg_print("Steer: ", steer)
        dbg_print("Speed: ", self.xvel)
        dbg_print("Is at wall: ", is_at_wall)
        dbg_print("Max speed: ", self.max_vel)
        dbg_print("StSpC: ", steer_speed_coefficient)
        dbg_print("Target: ", choosen_index)
        dbg_print("Gaps: ", len(self.gap_list))
        for gap in self.gap_list:
            minv = min(fscan[gap.start : gap.end])
            dbg_print(
                "Gap: ",
                str(gap.start)
                + " ,"
                + str(gap.end)
                + " | min:"
                + str(minv)
                + " max: "
                + str(gap.max_depth),
            )

        # ---Objects deletion-----
        self.gap_list.clear()


def main(args=None):
    rclpy.init(args=args)

    degf = DEGF()

    rclpy.spin(degf)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    degf.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

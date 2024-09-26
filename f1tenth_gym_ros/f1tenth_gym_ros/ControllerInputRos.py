from numpy import clip
import rclpy
import os
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy


def dbg_print(msg: str, var):
    print(f"{msg}{str(var)}", flush=True)


def lerp(a: float, b: float, t: float) -> float:
    return (1 - t) * a + t * b


def inv_lerp(a: float, b: float, v: float) -> float:
    return (v - a) / (b - a)


def remap(i_min: float, i_max: float, o_min: float, o_max: float, v: float) -> float:
    return lerp(o_min, o_max, inv_lerp(i_min, i_max, v))


class ControllerInput(Node):
    def __init__(self):
        super().__init__("controllerinput")
        self.subscription = self.create_subscription(
            Joy, "joy", self.listener_callback, 10
        )
        self.max_v = 5.0
        self.max_steer = 0.1
        self.max_steer_v = 0.4
        self.publisher = self.create_publisher(AckermannDriveStamped, "drive", 10)

    def listener_callback(self, msg):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = self.max_steer * msg.axes[0]
        drive_msg.drive.steering_angle_velocity = self.max_steer_v
        drive_msg.drive.speed = remap(
            -1.0, 1.0, -self.max_v, self.max_v, max(msg.axes[4], 0.0)
        )
        drive_msg.drive.acceleration = 0.4
        drive_msg.drive.jerk = 0.6
        self.publisher.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)

    controllerinput = ControllerInput()

    rclpy.spin(controllerinput)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controllerinput.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

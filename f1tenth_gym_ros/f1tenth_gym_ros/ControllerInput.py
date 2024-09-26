from math import radians
from f1tenth_gym_ros.DEGF import remap
import rclpy
import os
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from pyPS4Controller.controller import Controller


def dbg_print(msg: str, var):
    print(f"{msg}{str(var)}", flush=True)


class ControllerInput(Node):
    class F1Joy(Controller):

        def __init__(self, parent, **kwargs):
            Controller.__init__(self, **kwargs)
            self.parent = parent
            self.max_steering = radians(8.0)
            self.max_steering_vel = 0.1
            self.max_vel = 8.0
            self.input_max = 32766

        def on_R2_press(self, value):
            # print("on_R2_press: {}".format(value))
            if value > self.input_max * 9.0 / 10.0:
                return
            self.parent.drive_msg.drive.speed = remap(
                -self.input_max, self.input_max, 0, self.max_vel, value
            )
            self.parent.drive_msg.drive.acceleration = 11.4
            self.parent.drive_msg.drive.jerk = 2.3
            self.parent.publisher.publish(self.parent.drive_msg)

        def on_R2_release(self):
            self.parent.drive_msg.drive.speed = 0.0
            self.parent.drive_msg.drive.acceleration = 11.4
            self.parent.drive_msg.drive.jerk = 2.3
            self.parent.publisher.publish(self.parent.drive_msg)

        def on_L2_press(self, value):
            self.parent.drive_msg.drive.speed = 0.0
            self.parent.publisher.publish(self.parent.drive_msg)

        def on_L3_left(self, value):
            if abs(value) < self.input_max * 1.0 / 10.0 and 1:
                return
            turn = remap(
                0.0,
                -self.input_max,
                0.0,
                self.max_steering,
                value,
            )

            self.parent.drive_msg.drive.steering_angle = turn
            self.parent.drive_msg.drive.steering_angle_velocity = self.max_steering_vel

            if self.parent.drive_msg.drive.steering_angle < -self.max_steering and 0:
                self.parent.drive_msg.drive.steering_angle = -self.max_steering

            self.parent.publisher.publish(self.parent.drive_msg)

        def on_L3_right(self, value):
            if abs(value) < self.input_max * 1.0 / 10.0 and 1:
                return

            turn = remap(
                0.0,
                self.input_max,
                0.0,
                -self.max_steering,
                value,
            )

            self.parent.drive_msg.drive.steering_angle = turn
            self.parent.drive_msg.drive.steering_angle_velocity = self.max_steering_vel

            if self.parent.drive_msg.drive.steering_angle > self.max_steering and 0:
                self.parent.drive_msg.drive.steering_angle = self.max_steering

            self.parent.publisher.publish(self.parent.drive_msg)

        def on_L3_x_at_rest(self):
            self.parent.drive_msg.drive.steering_angle = 0.0
            self.parent.drive_msg.drive.steering_angle_velocity = self.max_steering_vel
            self.parent.publisher.publish(self.parent.drive_msg)

    def __init__(self):
        super().__init__("controllerinput")
        self.controller = self.F1Joy(
            interface="/dev/input/js0", connecting_using_ds4drv=False, parent=self
        )
        self.subscription = self.create_subscription(
            Odometry, "ego_racecar/odom", self.listener_callback, 10
        )
        self.angular_vely_odom = 0
        self.drive_msg = AckermannDriveStamped()
        self.publisher = self.create_publisher(AckermannDriveStamped, "drive", 10)
        self.controller.listen(timeout=5)

    def listener_callback(self, msg):
        self.angular_vely_odom = msg.twist.twist.angular.y
        dbg_print("Angular: ", str(self.angular_vely_odom))


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

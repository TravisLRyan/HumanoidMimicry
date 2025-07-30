import rclpy
from rclpy.node import Node
import numpy as np
import pinocchio as pin
import time
from geometry_msgs.msg import TransformStamped
from pose_tracker.robot_arm_tr import G1_29_ArmController
from pose_tracker.robot_arm_ik import G1_29_ArmIK
from unitree_go.msg import WirelessController

class ControllerNode(Node):
    def __init__(self):
        super().__init__('pose_follower_node')

        self.get_logger().info("Initializing ControllerNode...")
        self.arm_controller = G1_29_ArmController(simulation_mode=True)
        self.get_logger().info("Initialized ArmController")
        self.arm_ik = G1_29_ArmIK(Unit_Test=True, Visualization=False)
        self.get_logger().info("Initialized ArmIK")

        self.deadman_active = False
        self.wirelesscontroller = self.create_subscription(
            WirelessController,
            '/wirelesscontroller',
            self.controller_callback,
            10
        )

        self.r_hand_sub = self.create_subscription(
            TransformStamped,
            '/r_hand_cleaned',
            self.r_hand_callback,
            10
        )

        self.l_hand_sub = self.create_subscription(
            TransformStamped,
            '/l_hand_cleaned',
            self.l_hand_callback,
            10
        )

        self.latest_r_hand = None
        self.latest_l_hand = None

        self.arm_controller.speed_gradual_max()

        self.L_tf_target = pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([0.25, +0.25, 0.1]))
        self.R_tf_target = pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([0.25, -0.25, 0.1]))
        self.rotation_speed = 0.005
        self.step = 0

        self.timer = self.create_timer(0.01, self.update)

    def r_hand_callback(self, msg):
        q = msg.transform.rotation
        t = msg.transform.translation
        self.latest_r_hand = pin.SE3(
            pin.Quaternion(q.w, q.x, q.y, q.z),
            np.array([t.x, t.y, t.z])
        )

    def l_hand_callback(self, msg):
        q = msg.transform.rotation
        t = msg.transform.translation
        self.latest_l_hand = pin.SE3(
            pin.Quaternion(q.w, q.x, q.y, q.z),
            np.array([t.x, t.y, t.z])
        )

    def controller_callback(self, msg):
        self.deadman_active = (msg.keys == 48)

    def slerp(self, q1: pin.Quaternion, q2: pin.Quaternion, t: float) -> pin.Quaternion:
        q1.normalize()
        q2.normalize()
        dot = np.clip(q1.dot(q2), -1.0, 1.0)
        
        if dot < 0.0:
            q2_coeffs = q2.coeffs().copy()  # Create a writable copy of the coefficients
            q2_coeffs[:] *= -1  # Modify the copied coefficients
            q2 = pin.Quaternion(q2_coeffs)  # Recreate the quaternion with the modified coefficients
            dot = -dot
        
        if dot > 0.9995:
            result = q1.coeffs() * (1 - t) + q2.coeffs() * t
            return pin.Quaternion(result / np.linalg.norm(result))
        
        theta_0 = np.arccos(dot)
        theta = theta_0 * t
        q3 = (q2.coeffs() - q1.coeffs() * dot)
        q3 /= np.linalg.norm(q3)
        result = q1.coeffs() * np.cos(theta) + q3 * np.sin(theta)
        return pin.Quaternion(result)

    def step_towards_target(self, current: pin.SE3, target: pin.SE3, max_step=0.01):
        direction = target.translation - current.translation
        dist = np.linalg.norm(direction)
        if dist > max_step:
            direction = direction / dist * max_step
        new_translation = current.translation + direction

        q_current = pin.Quaternion(current.rotation)
        q_target = pin.Quaternion(target.rotation)
        q_interp = self.slerp(q_current, q_target, t=0.1)

        return pin.SE3(current.rotation, new_translation)

    def update(self):
        if self.deadman_active:
            if self.latest_l_hand is not None and self.latest_r_hand is not None:
                self.L_tf_target = self.step_towards_target(self.L_tf_target, self.latest_l_hand)
                self.R_tf_target = self.step_towards_target(self.R_tf_target, self.latest_r_hand)

            q = self.arm_controller.get_current_dual_arm_q()
            dq = self.arm_controller.get_current_dual_arm_dq()

            sol_q, sol_tauff = self.arm_ik.solve_ik(
                self.L_tf_target.homogeneous,
                self.R_tf_target.homogeneous,
                q, dq
            )

            self.arm_controller.ctrl_dual_arm(sol_q, sol_tauff)

            self.step += 1

def main():
    rclpy.init()
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

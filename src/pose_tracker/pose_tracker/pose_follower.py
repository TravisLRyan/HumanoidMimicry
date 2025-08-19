import rclpy
from rclpy.node import Node
import numpy as np
import pinocchio as pin
import time
import pygame
from geometry_msgs.msg import TransformStamped
from pose_tracker.robot_arm_tr import G1_29_ArmController
from pose_tracker.robot_arm_ik import G1_29_ArmIK
from unitree_go.msg import WirelessController

class ControllerNode(Node):
    def __init__(self):
        super().__init__('pose_follower_node')

        self.get_logger().info("Initializing Controller Node...")
        self.arm_controller = G1_29_ArmController(motion_mode = False, simulation_mode=True, freeze_legs=True)
        self.get_logger().info("Initialized ArmController")
        self.arm_ik = G1_29_ArmIK(Unit_Test=False, Visualization=False)
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
            '/pose_tracker/right_wrist',
            self.r_hand_callback,
            10
        )

        self.l_hand_sub = self.create_subscription(
            TransformStamped,
            '/pose_tracker/left_wrist',
            self.l_hand_callback,
            10
        )

        self.latest_r_hand = None
        self.latest_l_hand = None

        self.arm_controller.speed_gradual_max()

        self.L_tf_target = pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([0.25, +0.25, 0.5]))
        self.R_tf_target = pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([0.25, -0.25, 0.5]))


        # self.L_tf_target = pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([0.25, +0.25, 0.25]))
        # self.R_tf_target = pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([0.0, +0.25, 0.0]))

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

        # return pin.SE3(q_interp.toRotationMatrix(), new_translation)

        return pin.SE3(current.toRotationMatrix(), new_translation)
    
    def check_keys(self, x, y, z):
        step = 0.01
         # Refresh input state
        pygame.event.pump()
        keys = pygame.key.get_pressed()

        if keys[pygame.K_KP4]:
            y -= step
        if keys[pygame.K_KP6]:
            y += step
        if keys[pygame.K_KP2]:
            x -= step
        if keys[pygame.K_KP8]:
            x += step
        if keys[pygame.K_KP_PLUS]:
            z += step
        if keys[pygame.K_KP_MINUS]:
            z -= step

        return x, y, z


    def update(self):
        # if self.deadman_active:


        if self.latest_l_hand is not None and self.latest_r_hand is not None:
            self.L_tf_target = self.step_towards_target(self.L_tf_target, self.latest_l_hand)
            self.R_tf_target = self.step_towards_target(self.R_tf_target, self.latest_r_hand)

        rx, ry, rz = self.R_tf_target.translation
        lx, ly, lz = self.L_tf_target.translation

        # print("rx, ry, rz:", rx, ry, rz)

        rx, ry, rz = self.check_keys(rx, ry, rz)
        lx, ly, lz = self.check_keys(lx, ly, lz)

        self.R_tf_target.translation = np.array([rx, ry, rz])
        self.L_tf_target.translation = np.array([lx, ly, lz])

        q = self.arm_controller.get_current_dual_arm_q()
        dq = self.arm_controller.get_current_dual_arm_dq()

        #old
        sol_q, sol_tauff = self.arm_ik.solve_ik(
            self.L_tf_target.homogeneous,
            self.R_tf_target.homogeneous,
            q, dq
        )

        self.arm_controller.ctrl_dual_arm(sol_q, sol_tauff)

        # #new

        #TESTING NEW IK SOLVE

        # self.L_tf_target.homogeneous = 

        #END TESTING NEW IK SOLVE

        # try:
        #     sol_q, sol_tauff = self.arm_ik.solve_ik(
        #         self.L_tf_target.homogeneous,
        #         self.R_tf_target.homogeneous,
        #         q, dq
        #     )
        # except Exception as e:
        #     self.get_logger().warn(f"IK solve failed: {e}")
        #     return  # skip this tick

        # # Reject non-finite or outrageous values
        # if sol_q is None or not np.all(np.isfinite(sol_q)):
        #     self.get_logger().warn("IK returned non-finite joint solution; skipping tick")
        #     return

        # # Conservative joint clamp (tune per joint later)
        # sol_q = np.clip(sol_q, -2.8, 2.8)

        # # Extra safety: rate-limit command (0.8 rad/s @ 100 Hz → 0.008 rad/step)
        # dt = 0.01
        # max_joint_vel = 0.8
        # max_step = max_joint_vel * dt
        # dq_cmd = np.clip(sol_q - q, -max_step, max_step)
        # cmd_q = q + dq_cmd

        # # Sanitize torques
        # if sol_tauff is None:
        #     sol_tauff = np.zeros_like(cmd_q)
        # else:
        #     sol_tauff = np.nan_to_num(sol_tauff, nan=0.0, posinf=0.0, neginf=0.0)
        #     sol_tauff = np.clip(sol_tauff, -10.0, 10.0)

        # self.arm_controller.ctrl_dual_arm(cmd_q, sol_tauff)
        # print(f"Step {self.step}: L_target={self.L_tf_target.translation}, R_target={self.R_tf_target.translation}, cmd_q={cmd_q}, sol_tauff={sol_tauff}")
        # # self.arm_controller.ctrl_dual_arm(np.zeros(14), np.zeros(14))  # Placeholder for actual command
        # # #endnew

        self.step += 1

def main():
    pygame.init()
    screen = pygame.display.set_mode((200, 200))
    rclpy.init()
    node = ControllerNode()
    time.sleep(5.0)  # Allow time for subscriptions to initialize
    node.get_logger().info("Moving to Home Pose.")
    node.arm_controller.ctrl_dual_arm_go_home()
    time.sleep(2.0)  # Allow time for the robot to reach home position
    node.get_logger().info("Moved to Home Pose. Starting pose follower.")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

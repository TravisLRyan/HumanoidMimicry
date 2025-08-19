import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import sensor_msgs_py.point_cloud2 as pc2

import depthai as dai
import numpy as np
import mediapipe as mp
import open3d as o3d
import cv2
import time
import sys

# Added imports for transforms
from geometry_msgs.msg import TransformStamped
import tf2_ros

FPS = 30

class FPSCounter:
    def __init__(self):
        self.frameCount = 0
        self.fps = 0
        self.startTime = time.time()

    def tick(self):
        self.frameCount += 1
        if self.frameCount % 10 == 0:
            elapsedTime = time.time() - self.startTime
            self.fps = self.frameCount / elapsedTime
            self.frameCount = 0
            self.startTime = time.time()
        return self.fps

class KalmanFilter3D:
    def __init__(self, dt=1/30, process_var=1e-2, meas_var=1e-1):
        self.dt = dt
        self.A = np.eye(6)
        for i in range(3):
            self.A[i, i+3] = dt

        self.H = np.hstack((np.eye(3), np.zeros((3, 3))))
        self.Q = process_var * np.eye(6)
        self.R = meas_var * np.eye(3)

        self.P = np.eye(6)
        self.x = np.zeros((6, 1))
        self.initialized = False

    def predict(self):
        self.x = self.A @ self.x
        self.P = self.A @ self.P @ self.A.T + self.Q

    def update(self, z):
        z = z.reshape(3, 1)
        if not self.initialized:
            self.x[:3] = z
            self.x[3:] = 0
            self.initialized = True

        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x += K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

    def get_position(self):
        return self.x[:3].flatten()

# Depth window averaging function
def average_depth_window(points, x, y, width, height, kernel=3, fallback=None):
    half_k = kernel // 2
    xs = np.clip(np.arange(x - half_k, x + half_k + 1), 0, width - 1)
    ys = np.clip(np.arange(y - half_k, y + half_k + 1), 0, height - 1)
    pts = []
    for yy in ys:
        for xx in xs:
            idx = yy * width + xx
            pt = points[idx]
            if np.all(np.isfinite(pt)) and pt[2] > 0:
                pts.append(pt)
    if pts:
        return np.mean(pts, axis=0)
    return fallback

def getPosePoints(frame, pose_model):
    results = pose_model.process(frame)
    if not results.pose_landmarks:
        return []
    return [(int(lm.x * frame.shape[1]), int(lm.y * frame.shape[0])) for lm in results.pose_landmarks.landmark]

def drawPose(frame, poses):
    for pose in poses:
        cv2.circle(frame, pose, 3, (0, 255, 0), -1)

class PoseTrackerNode(Node):
    def __init__(self, visualise=False):
        super().__init__('pose_tracker_node')
        self.get_logger().info("Starting Pose Tracker Node")
        self.pcl_left_pub = self.create_publisher(PointCloud2, '/pose_tracker/left_wrist', 10)
        self.pcl_right_pub = self.create_publisher(PointCloud2, '/pose_tracker/right_wrist', 10)

        # Add tf broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.visualise = visualise

        self.pose = mp.solutions.pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            smooth_landmarks=True,
            enable_segmentation=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        self.pipeline = self.create_pipeline()
        self.device = dai.Device(self.pipeline)
        self.q = self.device.getOutputQueue(name="out", maxSize=4, blocking=False)
        if self.visualise:
            self.vis = o3d.visualization.VisualizerWithKeyCallback()
            self.vis.create_window()
            self.vis.register_key_action_callback(81, self.stop)
            self.pcd = o3d.geometry.PointCloud()
            self.coordinateFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1000, origin=[0,0,0])
            self.vis.add_geometry(self.coordinateFrame)

            self.keypoint_spheres = {}
            self.keypoint_colors = {}
        self.keypoint_positions = {}
        self.left_wrist_kf = KalmanFilter3D()
        self.right_wrist_kf = KalmanFilter3D()
        

        self.fpsCounter = FPSCounter()
        self.isRunning = True

        self.run_loop()

    def pelvis_relative(self, joint, pelvis_center):
        return joint - pelvis_center

    def stop(self, vis, action, mods):
        if action == 0:
            self.isRunning = False

    def create_pipeline(self):
        pipeline = dai.Pipeline()
        camRgb = pipeline.create(dai.node.ColorCamera)
        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoRight = pipeline.create(dai.node.MonoCamera)
        depth = pipeline.create(dai.node.StereoDepth)
        pointcloud = pipeline.create(dai.node.PointCloud)
        sync = pipeline.create(dai.node.Sync)
        xOut = pipeline.create(dai.node.XLinkOut)
        xOut.setStreamName("out")

        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        camRgb.setIspScale(1,3)
        camRgb.setFps(FPS)

        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setCamera("left")
        monoLeft.setFps(FPS)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setCamera("right")
        monoRight.setFps(FPS)

        depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DETAIL)
        depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        depth.setLeftRightCheck(True)
        depth.setExtendedDisparity(False)
        depth.setSubpixel(True)
        depth.setDepthAlign(dai.CameraBoardSocket.CAM_A)

        monoLeft.out.link(depth.left)
        monoRight.out.link(depth.right)
        depth.depth.link(pointcloud.inputDepth)
        camRgb.isp.link(sync.inputs["rgb"])
        pointcloud.outputPointCloud.link(sync.inputs["pcl"])
        sync.out.link(xOut.input)

        return pipeline

    def direction_to_quaternion(self, direction):
        """
        Returns a quaternion (x, y, z, w) that rotates [0, 0, 1] to align with the given direction vector.
        """
        import numpy as np
        direction = np.array(direction)
        direction = direction / np.linalg.norm(direction)
        z_axis = np.array([0, 0, 1])
        v = np.cross(z_axis, direction)
        c = np.dot(z_axis, direction)
        s = np.linalg.norm(v)
        if s == 0:
            return [0, 0, 0, 1] if c > 0 else [1, 0, 0, 0]  # identity or 180 deg flip
        axis = v / s
        angle = np.arctan2(s, c)
        qw = np.cos(angle / 2)
        qx, qy, qz = axis * np.sin(angle / 2)
        return [qx, qy, qz, qw]
    #!/home/parallels/miniconda3/envs/humanoid/bin/python

    def rotate_about_z(self, q, angle_rad):
        """
        Rotate quaternion q by angle_rad about the Z axis in the base (map/pelvis) frame.
        """
        import numpy as np
        half_angle = angle_rad / 2
        qz = [0, 0, np.sin(half_angle), np.cos(half_angle)]
        # Quaternion multiplication q' = q * qz (rotation in base frame)
        x1, y1, z1, w1 = q
        x2, y2, z2, w2 = qz
        return [
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2
        ]

    def has_nan(self, tf):
        return np.isnan(tf.transform.translation.x) or \
            np.isnan(tf.transform.translation.y) or \
            np.isnan(tf.transform.translation.z) or \
            np.isnan(tf.transform.rotation.x) or \
            np.isnan(tf.transform.rotation.y) or \
            np.isnan(tf.transform.rotation.z) or \
            np.isnan(tf.transform.rotation.w)

    def rotate_about_z(self, q, angle_rad):
        import numpy as np
        half_angle = angle_rad / 2
        qz = [0, 0, np.sin(half_angle), np.cos(half_angle)]
        # Quaternion multiplication q' = qz * q
        x1, y1, z1, w1 = qz
        x2, y2, z2, w2 = q
        return [
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2
        ]


    def camera_to_pelvis(self, vec):
        # Converts from camera frame (+Z forward, +X right, +Y down)
        # to pelvis frame (+Z up, +X forward, +Y left), with flipped directions
        x_c, y_c, z_c = vec
        x_p = -z_c       # forward (flipped)
        y_p = x_c        # left (flipped)
        z_p = y_c        # up (flipped)
        return np.array([x_p, y_p, z_p])

    def run_loop(self):
        while self.isRunning:
            inMessage = self.q.get()
            frame = inMessage["rgb"].getCvFrame()
            pcl = inMessage["pcl"]
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            fps = self.fpsCounter.tick()
            cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            keypoints = getPosePoints(frame, self.pose)
            drawPose(frame, keypoints)
            cv2.imshow("Pose View", frame)

            if pcl:
                points = pcl.getPoints().astype(np.float64)
                                
                width, height = rgb.shape[1], rgb.shape[0]
                visible = set()

                for idx, (x, y) in enumerate(keypoints):
                    if idx not in (13, 14, 15, 16, 23, 24):  # only include left and right hand keypoints
                        continue
                    if 0 <= x < width and 0 <= y < height:
                        # Use averaged depth over a 3x3 window, fallback to previous value
                        prev_pt = self.keypoint_positions.get(idx, None)
                        pt = average_depth_window(points, x, y, width, height, kernel=3, fallback=prev_pt)
                        if pt is None:
                            continue
                        if idx == 15:
                            self.left_wrist_kf.predict()
                            self.left_wrist_kf.update(pt)
                            pt = self.left_wrist_kf.get_position()
                        elif idx == 16:
                            self.right_wrist_kf.predict()
                            self.right_wrist_kf.update(pt)
                            pt = self.right_wrist_kf.get_position()

                        self.keypoint_positions[idx] = pt
                        if self.visualise:
                            color = rgb[y, x] / 255.0
                            visible.add(idx)
                            if idx not in self.keypoint_spheres:
                                s = o3d.geometry.TriangleMesh.create_sphere(radius=20)
                                s.paint_uniform_color(color)
                                s.translate(pt - np.array([10, 10, 10]))
                                self.keypoint_spheres[idx] = s
                                self.keypoint_positions[idx] = pt
                                self.keypoint_colors[idx] = color
                                self.vis.add_geometry(s)
                            else:
                                delta = pt - self.keypoint_positions[idx]
                                self.keypoint_spheres[idx].translate(delta)
                                self.keypoint_positions[idx] = pt
                                self.vis.update_geometry(self.keypoint_spheres[idx])
                if self.visualise:
                    for idx in set(self.keypoint_spheres.keys()) - visible:
                        self.vis.remove_geometry(self.keypoint_spheres[idx])
                        del self.keypoint_spheres[idx]
                        del self.keypoint_positions[idx]
                        del self.keypoint_colors[idx]
            
            if self.visualise:
                self.vis.poll_events()
                self.vis.update_renderer()


            #publish keypoints
            header = std_msgs.msg.Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "map"  #<-- tf frame
            self.keypoint_positions = {idx: val / 1000.0 for idx, val in self.keypoint_positions.items()}  # convert mm to m

            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
            ]
            left_point = []
            right_point = []
            for idx, pt in self.keypoint_positions.items():
                if idx not in (15, 16):  # only include left and right hand keypoints
                # if idx != 15 and idx != 16:  # only include left and right hand keypoints
                    continue
                z = pt[2]
                # Normalize z value between 0 and 1 (adjust range as needed)
                z_norm = max(0.0, min(1.0, (z - 0.0) / 2.0))
                r = int(z_norm * 255)
                g = 0
                b = 255 - r
                rgb = (r << 16) | (g << 8) | b
                if idx == 15:
                    left_point.append([pt[0], pt[1], pt[2], rgb])
                elif idx == 16:
                    right_point.append([pt[0], pt[1], pt[2], rgb])

            pcl_left_msg = pc2.create_cloud(header, fields, left_point)
            pcl_right_msg = pc2.create_cloud(header, fields, right_point)
            self.pcl_left_pub.publish(pcl_left_msg)
            self.pcl_right_pub.publish(pcl_right_msg)

            # Calculate and publish transforms for elbow-to-wrist vectors
            now = self.get_clock().now().to_msg()

            if 23 in self.keypoint_positions and 24 in self.keypoint_positions:
                left_hip = self.keypoint_positions[23]
                right_hip = self.keypoint_positions[24]
                pelvis_center = 0.5 * (left_hip + right_hip)
            else:
                self.get_logger().warn("Hip keypoints missing, skipping frame.")
                continue

            if 13 in self.keypoint_positions and 15 in self.keypoint_positions:
                l_elbow = self.pelvis_relative(self.keypoint_positions[13], pelvis_center)
                l_wrist = self.pelvis_relative(self.keypoint_positions[15], pelvis_center)
                l_vec = self.camera_to_pelvis(l_wrist - l_elbow)
                l_vec /= np.linalg.norm(l_vec)

                l_tf = TransformStamped()
                l_tf.header.stamp = now
                l_tf.header.frame_id = "pelvis"
                l_tf.child_frame_id = "left_wrist_vector"
                l_trans = self.camera_to_pelvis(l_wrist)
                l_tf.transform.translation.x = l_trans[0]
                l_tf.transform.translation.y = l_trans[1]
                l_tf.transform.translation.z = l_trans[2]

                # Compute quaternion to align z-axis with l_vec
                l_qx, l_qy, l_qz, l_qw = map(float, self.direction_to_quaternion(l_vec))
                l_tf.transform.rotation.x = l_qx
                l_tf.transform.rotation.y = l_qy
                l_tf.transform.rotation.z = l_qz
                l_tf.transform.rotation.w = l_qw
                
                if not self.has_nan(l_tf):
                    self.tf_broadcaster.sendTransform(l_tf)
                    self.get_logger().info("Published left wrist vector transform")
                else:
                    self.get_logger().warn("Skipping left wrist transform due to NaN values")
            

            if 14 in self.keypoint_positions and 16 in self.keypoint_positions:
                r_elbow = self.pelvis_relative(self.keypoint_positions[14], pelvis_center)
                r_wrist = self.pelvis_relative(self.keypoint_positions[16], pelvis_center)
                r_vec = self.camera_to_pelvis(r_wrist - r_elbow)
                r_vec /= np.linalg.norm(r_vec)

                r_tf = TransformStamped()
                r_tf.header.stamp = now
                r_tf.header.frame_id = "pelvis"
                r_tf.child_frame_id = "right_wrist_vector"
                r_trans = self.camera_to_pelvis(r_wrist)
                r_tf.transform.translation.x = r_trans[0]
                r_tf.transform.translation.y = r_trans[1]
                r_tf.transform.translation.z = r_trans[2]

                # Compute quaternion to align z-axis with r_vec
                r_qx, r_qy, r_qz, r_qw = map(float, self.direction_to_quaternion(r_vec))
                r_tf.transform.rotation.x = r_qx
                r_tf.transform.rotation.y = r_qy
                r_tf.transform.rotation.z = r_qz
                r_tf.transform.rotation.w = r_qw
                

                if not self.has_nan(r_tf):
                    self.tf_broadcaster.sendTransform(r_tf)
                    self.get_logger().info("Published right wrist vector transform")
                else:
                    self.get_logger().warn("Skipping right wrist transform due to NaN values")
            
            if cv2.waitKey(1) == ord('q'):
                break

        if self.visualise:
            self.vis.destroy_window()
            self.get_logger().info("Shutting down Pose Tracker")

def main(args=None):
    rclpy.init(args=args)
    node = PoseTrackerNode(visualise=False)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
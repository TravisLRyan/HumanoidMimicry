import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
# from tf2_ros.transform_exception import TransformException

class PointCloudTransformer(Node):
    def __init__(self):
        super().__init__('pointcloud_transformer')
        self.declare_parameter('input_topic', '/pose_tracker/points')
        self.declare_parameter('output_topic', '/points_in_pelvis')
        self.declare_parameter('target_frame', 'pelvis')
        
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        self.sub = self.create_subscription(PointCloud2, input_topic, self.callback, 10)
        self.pub = self.create_publisher(PointCloud2, output_topic, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                # msg.header.frame_id, #change this to 'map' if you want to transform from 'map' frame
                'map', #only for bagdata
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            transformed_cloud = do_transform_cloud(msg, transform)
            self.pub.publish(transformed_cloud)

        except TransformException as ex:
            self.get_logger().warn(f'Transform failed: {ex}')

def main():
    rclpy.init()
    node = PointCloudTransformer()
    rclpy.spin(node)
    rclpy.shutdown()

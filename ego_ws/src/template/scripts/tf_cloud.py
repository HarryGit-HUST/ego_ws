#!/usr/bin/env python3

import rospy
import tf2_ros
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_sensor_msgs import do_transform_cloud  # ✅ 正确导入

class LivoxToPointCloud2:
    def __init__(self):
        rospy.init_node('livox_to_pointcloud2', anonymous=True)
        self.lidar_topic = rospy.get_param('~lidar_topic', '/livox/lidar')
        self.output_topic = rospy.get_param('~output_topic', '/cloud_registered')
        self.target_frame = rospy.get_param('~target_frame', 'world')
        self.publish_map_to_world = rospy.get_param('~publish_map_to_world', True)
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.world_frame = rospy.get_param('~world_frame', 'world')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=10)
        self.sub = rospy.Subscriber(self.lidar_topic, CustomMsg, self.livox_callback)

        if self.publish_map_to_world:
            self.publish_map_world_tf()

    def publish_map_world_tf(self):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = rospy.Time.now()
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = self.world_frame
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = 0.0
        tf_msg.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(tf_msg)
        rospy.loginfo(f"Published static TF: {self.map_frame} -> {self.world_frame}")

    def livox_callback(self, msg):
        if not msg.points:
            return

        points = []
        for p in msg.points:
            points.append([p.x, p.y, p.z, p.reflectivity])

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1)
        ]

        pc2_msg = pc2.create_cloud(header, fields, points)

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                msg.header.stamp,
                rospy.Duration(0.1)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF lookup failed: {e}")
            return

        # ✅ 正确调用
        pc2_transformed = do_transform_cloud(pc2_msg, transform)
        self.pub.publish(pc2_transformed)

if __name__ == '__main__':
    try:
        node = LivoxToPointCloud2()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
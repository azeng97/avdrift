#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import message_filters
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import Imu


def handle_pose(hedge, imu):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "drift_car/base_link"
    t.transform.translation.x = hedge.pose.pose.position.x - init_pos.pose.pose.position.x
    t.transform.translation.y = hedge.pose.pose.position.y - init_pos.pose.pose.position.y
    t.transform.translation.z = hedge.pose.pose.position.z - init_pos.pose.pose.position.z

    q1_inv = [init_pos.pose.pose.orientation.x, init_pos.pose.pose.orientation.y, init_pos.pose.pose.orientation.z, -init_pos.pose.pose.orientation.w]

    q2 = [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w]
    qr = tf.transformations.quaternion_multiply(q2, q1_inv)
    t.transform.rotation.x = qr[0]
    t.transform.rotation.y = qr[1]
    t.transform.rotation.z = qr[2]
    t.transform.rotation.w = qr[3]
    br.sendTransform(t)

def init_map(init_hedge, init_imu):
    init_pos.pose.pose.position = init_hedge.pose.pose.position
    init_pos.pose.pose.orientation = init_imu.orientation


if __name__ == "__main__":

    rospy.init_node("tf2_drift_publisher")

    init_pos = PoseWithCovarianceStamped()

    init_hedge = rospy.wait_for_message("/hedge_pose", PoseWithCovarianceStamped)
    init_imu = rospy.wait_for_message("/imu", Imu)
    rospy.loginfo("Received Initial Hedge Pose")
    init_map(init_hedge, init_imu)

    hedge_sub = message_filters.Subscriber("/hedge_pose", PoseWithCovarianceStamped)
    imu_sub = message_filters.Subscriber("/imu", Imu)
    ts = message_filters.ApproximateTimeSynchronizer([hedge_sub, imu_sub], 10, 0.1)
    ts.registerCallback(handle_pose)
    rospy.spin()
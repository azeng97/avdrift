import rospy
import tf
from std_msgs.msg import Float64, String
from std_srvs.srv import Empty
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from marvelmind_nav.msg import hedge_pos_ang


def main():
    global hedge_pose_pub
    global out_pose
    rospy.init_node("hedgehog_node")
    rospy.Subscriber("hedge_pos_ang", hedge_pos_ang, pos_callback)
    hedge_pose_pub = rospy.Publisher("hedge_pose", PoseWithCovarianceStamped, queue_size=10)
    out_pose = PoseWithCovarianceStamped()
    out_pose.header.frame_id = "drift_car/beacon_link"
    rospy.spin()

def pos_callback(data):
    out_pose.header.stamp = rospy.Time.now()
    out_pose.pose.pose.position.x = data.x_m
    out_pose.pose.pose.position.y = data.y_m
    out_pose.pose.pose.position.z = data.z_m
    hedge_pose_pub.publish(out_pose)

if __name__ == "__main__":
    main()
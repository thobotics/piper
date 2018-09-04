#!/usr/bin/env python

import tf
import rospy
from geometry_msgs.msg import Pose, PoseArray
from trajectory_msgs.msg import JointTrajectory

est_pub_ = rospy.Publisher('/est_traj', PoseArray, queue_size=10)
plan_pub_ = rospy.Publisher('/plan_traj', PoseArray, queue_size=10)

def traj2posearray_(traj):
    pose_array = PoseArray()
    pose_array.header.frame_id = "/map"
    for point in traj:
        quaternion = tf.transformations.quaternion_from_euler(0.,0., point.positions[2])

        pose = Pose()
        pose.position.x = point.positions[0]
        pose.position.y = point.positions[1]
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        pose_array.poses.append(pose)

    return pose_array

def est_traj_callback(data):
    pose_array = traj2posearray_(data.points)
    est_pub_.publish(pose_array)

def plan_traj_callback(data):
    pose_array = traj2posearray_(data.points)
    plan_pub_.publish(pose_array)

def run():
    rospy.init_node('steap_visualize', anonymous=True)

    # Create subscriber
    est_sub = rospy.Subscriber("/piper/est_traj", JointTrajectory, est_traj_callback)
    plan_sub = rospy.Subscriber("/piper/plan_traj", JointTrajectory, plan_traj_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass

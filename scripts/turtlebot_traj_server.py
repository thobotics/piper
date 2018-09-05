#! /usr/bin/env python

import rospy
import tf
import actionlib

import control_msgs.msg
from geometry_msgs.msg import Pose, Twist, PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

base_pose_pub_ = None
arm_pub_ = None
listener_ = None

class TrajAction(object):
    # create messages that are used to publish feedback/result
    _feedback = control_msgs.msg.FollowJointTrajectoryActionFeedback()
    _result = control_msgs.msg.FollowJointTrajectoryActionResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        self._cmd_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        self._listener = tf.TransformListener()

    def execute_cb(self, goal):
        # TODO: Load from parameters
        total_time = 20.0
        total_step = 20
        control_inter = 10

        step_time = total_time / total_step
        iter_time = step_time / control_inter

        # helper variables
        r = rospy.Rate(1/iter_time)
        success = True
        twist = Twist()

        for p in goal.trajectory.points:

            # # check that preempt has not been requested by the client
            # if self._as.is_preempt_requested():
            #     rospy.loginfo('%s: Preempted' % self._action_name)
            #     self._as.set_preempted()
            #     success = False
            #     break

            action = p.velocities[:3]

            twist.linear.x = action[0]
            twist.linear.y = action[1]
            twist.angular.z = action[2]

            self._cmd_pub.publish(twist)

            # self._feedback.feedback.desired = goal.trajectory.points
            # self._feedback.feedback.actual = goal.trajectory.points # TODO: Feedback real pose
            # # publish the feedback
            # self._as.publish_feedback(self._feedback.feedback)

            r.sleep()

        if success:
            self._result.result.error_code = self._result.result.SUCCESSFUL
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result.result)

def odom_cb(data):
    listener_.waitForTransform("/odom", "/map", rospy.Time(0), rospy.Duration(4.0))
    odom = PoseStamped()
    odom.header.frame_id = "odom"
    odom.header.stamp = rospy.Time(0)
    odom.pose = data.pose.pose
    map_pose = listener_.transformPose("map", odom)

    j_state = JointState()
    j_state.name = ["right_shoulder_pan_joint", "right_shoulder_lift_joint", "right_elbow_joint",
  "right_wrist_1_joint", "right_wrist_2_joint", "right_wrist_3_joint"]
    j_state.position = [0.,0.,0.,0.,0.,0.]
    j_state.velocity = [0.,0.,0.,0.,0.,0.]

    base_pose_pub_.publish(map_pose.pose)
    arm_pub_.publish(j_state)

if __name__ == '__main__':
    rospy.init_node('trajfeedback', anonymous=True)

    global base_pose_pub_, listener_
    base_pose_pub_ = rospy.Publisher('/base_state', Pose, queue_size=10)
    arm_pub_ = rospy.Publisher('/joint_states', JointState, queue_size=10)
    listener_ = tf.TransformListener()

    rospy.Subscriber("/odom", Odometry, odom_cb)
    server = TrajAction("/vector/full_body_controller/trajectory")
    rospy.spin()

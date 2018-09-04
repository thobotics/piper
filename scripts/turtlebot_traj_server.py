#! /usr/bin/env python

import rospy

import actionlib

import control_msgs.msg
from trajectory_msgs.msg import JointTrajectory

class TrajAction(object):
    # create messages that are used to publish feedback/result
    _feedback = control_msgs.msg.FollowJointTrajectoryActionFeedback()
    _result = control_msgs.msg.FollowJointTrajectoryActionResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self._count = 0

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1/3.)
        success = True

        for p in goal.trajectory.points:
            print(p.positions[:3])
            # print(p.velocities[:3])
            print("--------------------")
        self._count += 1
        print("Received callback", self._count)

        # r.sleep()
        # # append the seeds for the fibonacci sequence
        # self._feedback.sequence = []
        # self._feedback.sequence.append(0)
        # self._feedback.sequence.append(1)
        #
        # # publish info to the console for the user
        # rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))
        #
        # # start executing the action
        # for i in range(1, goal.order):
        #     # check that preempt has not been requested by the client
        #     if self._as.is_preempt_requested():
        #         rospy.loginfo('%s: Preempted' % self._action_name)
        #         self._as.set_preempted()
        #         success = False
        #         break
        #     self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
        #     # publish the feedback
        #     self._as.publish_feedback(self._feedback)
        #     # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        #     r.sleep()
        #
        # if success:
        #     self._result.sequence = self._feedback.sequence
        #     rospy.loginfo('%s: Succeeded' % self._action_name)
        #     self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('trajfeedback', anonymous=True)
    # server = TrajAction(rospy.get_name())
    server = TrajAction("/vector/full_body_controller/trajectory")
    rospy.spin()

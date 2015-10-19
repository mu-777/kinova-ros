#! /usr/bin/env python
"""A helper program to test cartesian goals for the JACO and MICO arms."""

import roslib;

roslib.load_manifest('jaco_demo')
import rospy
import tf2_ros

import sys
import numpy as np

import actionlib
from jaco_msgs.msg import ArmPoseAction, ArmPoseGoal
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Vector3, Quaternion, Transform, TransformStamped
from std_srvs.srv import Trigger, TriggerResponse

import goal_generators


def pose2transform(pose):
    pos, ori = pose.position, pose.orientation
    return Transform(translation=Vector3(x=pos.x, y=pos.y, z=pos.z),
                     rotation=ori)


class MICOCartesianPoseControl(object):
    def __init__(self):
        action_address = '/mico_arm_driver/arm_pose/arm_pose'
        self._client = actionlib.SimpleActionClient(action_address, ArmPoseAction)
        self._goal = ArmPoseGoal(pose=PoseStamped(header=Header(stamp=rospy.Time.now(),
                                                                frame_id='/mico_api_origin'),
                                                  pose=Pose(orientation=Quaternion(w=1.0))))
        self._action_trigger = False
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._ref_tf_stamped = TransformStamped(header=Header(stamp=rospy.Time.now(),
                                                              frame_id='mico_api_origin'),
                                                child_frame_id='ref_pose',
                                                transform=pose2transform(self._goal.pose.pose))
        rospy.Subscriber('/mico_ref_pose', Pose, self._ref_pose_callback)
        rospy.Service('/mico_action_trigger', Trigger, self._action_trigger_srv_handler)

    def _ref_pose_callback(self, input_pose):
        self._goal.pose.header.stamp = self._ref_tf_stamped.header.stamp = rospy.Time.now()
        self._goal.pose.pose = input_pose
        self._ref_tf_stamped.transform = pose2transform(input_pose)

    def _action_trigger_srv_handler(self, req):
        if self._action_trigger:
            self._client.cancel_all_goals()
        self._action_trigger = True
        return TriggerResponse(success=True, message='OK')

    def update(self):
        self._tf_broadcaster.sendTransform(self._ref_tf_stamped)
        if self._action_trigger:
            self._do_action()
            self._action_trigger = False

    def _do_action(self, duration=3.0):
        print("Do Action!")
        self._client.send_goal(self._goal,
                               done_cb=self._done_cb,
                               active_cb=self._active_cb,
                               feedback_cb=self._feedback_cb)

        # if self._client.wait_for_result(rospy.Duration(duration)):
        # return self._client.get_result()
        # else:
        # self._client.cancel_all_goals()
        # print('        the cartesian action timed-out')
        # return None

    def _done_cb(self, msg1, msg2):
        print('done_cb', type(msg1), type(msg2))
        print(msg1)
        print(msg2)

    def _active_cb(self):
        print('active_cb')

    def _feedback_cb(self, msg):
        print('feedback_cb', type(msg))
        print(msg)


# --------------------------------------------
if __name__ == '__main__':
    rospy.init_node('mico_action_test', anonymous=True)
    rate_mgr = rospy.Rate(100)  # Hz

    mico_action_test = MICOCartesianPoseControl()

    while not rospy.is_shutdown():
        mico_action_test.update()
        rate_mgr.sleep()

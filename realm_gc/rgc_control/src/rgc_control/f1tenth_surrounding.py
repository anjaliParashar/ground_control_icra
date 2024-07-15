#!/usr/bin/env python3
"""Define class for robot control with surroundings"""
import os

import numpy as np
import rospy

from rgc_control.robot_control import RobotControl
from rgc_state_estimators.msg import F1TenthState
from geometry_msgs.msg import TransformStamped

class F1TenthSurrounding(RobotControl):
    def __init__(self):
        super(F1TenthSurrounding, self).__init__()

        # Subscribe to state estimation topic from ros param
        self.state = None
        self.state_estimate_topic = rospy.get_param(
            "~state_estimate_topic", f"{rospy.get_name()}/estimate"
        )
        self.estimate_sub = rospy.Subscriber(
            self.state_estimate_topic, F1TenthState, self.state_estimate_callback
        )

        # Subscribe to surrounding vehicle state
        self.surrounding_user_id = rospy.get_param("~user_id")
        self.surrounding_state = None
        self.surrounding_estimate_topic = rospy.get_param(
            f"{self.surrounding_user_id}_estimation_node/estimate"
        )
        self.surrounding_estimate_pub = rospy.Subscriber(
            self.surrounding_estimate_topic, TransformStamped, self.surrounding_estimate_callback
        )

    def state_estimate_callback(self, msg):
        self.state = msg

    def surrounding_estimate_callback(self, msg):
        self.surrounding_state = msg

    def reset_control(self, msg=None):
        """Reset the control to stop the experiment and publish the command."""
        return

    def update(self):
        """
        Update and publish the control.
        This function implements and calls the control prediction and update steps.
        """
        # print(self.state)
        # print(self.surrounding_state)

        if self.state is not None:
            rospy.loginfo(f"f1tenth state:")
            rospy.loginfo(f" |----- x: {self.state.x}")
            rospy.loginfo(f" |----- y: {self.state.y}")
            rospy.loginfo(f" |- theta: {self.state.theta}")
            rospy.loginfo(f" |- speed: {self.state.theta}")

        if self.surrounding_state is not None:
            translation = self.surrounding_state.translation
            rotation = self.surrounding_state.rotation
            rospy.loginfo(f"surrounding state ({self.surrounding_user_id}):")
            rospy.loginfo(f" |- translation:")
            rospy.loginfo(f" |   |- x: {translation.x}")
            rospy.loginfo(f" |   |- y: {translation.y}")
            rospy.loginfo(f" |   |- z: {translation.z}")
            rospy.loginfo(f" |- rotation:")
            rospy.loginfo(f" |   |- x: {rotation.x}")
            rospy.loginfo(f" |   |- y: {rotation.y}")
            rospy.loginfo(f" |   |- z: {rotation.z}")
            rospy.loginfo(f" |   |- w: {rotation.w}")


if __name__ == "__main__":
    try:
        surrounding_node = F1TenthSurrounding()
        surrounding_node.run()
    except rospy.ROSInterruptException:
        pass

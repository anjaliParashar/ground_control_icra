#!/usr/bin/env python3
"""Define a state estimator for general objects tracked by vicon"""
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import TransformStamped

from rgc_state_estimators.state_estimator import StateEstimator


class ObjectStateEstimator(StateEstimator):
    """
    State estimator for general objects tracked by vicon
    """

    def __init__(self):
        # we need to use a different name for the node
        # super(ObjectStateEstimator, self).__init__()

        self.user_id = rospy.get_param("~user_id")

        # Initialize the ROS node
        rospy.init_node(f"{self.user_id}_estimation_node")

        # Fetch parameters
        self.rate = rospy.get_param("~rate", 10)  # Hz
        self.dt = 1.0 / self.rate
        self.rate_timer = rospy.Rate(self.rate)

        self.position_topic = rospy.get_param(
            "~position_topic", f"/vicon/{self.user_id}/{self.user_id}"
        )

        # Publisher
        self.estimate_pub = rospy.Publisher(
            f"{rospy.get_name()}/estimate", TransformStamped, queue_size=10
        )

        # Reset event subscriber
        self.reset_sub = rospy.Subscriber(
            f"{rospy.get_name()}/reset", Empty, self.reset_state
        )

        # Other subscribers
        self.position_msg = None
        self.position_sub = rospy.Subscriber(
            self.position_topic, TransformStamped, self.position_callback
        )

        # Initialize the state
        self.reset_state()
        
    def reset_state(self, msg=None):
        """Reset the state of the estimator."""
        return

    def update(self):
        """Update the filter state and publish the new state estimate."""
        if self.position_msg is not None:
            self.estimate_pub.publish(self.position_msg)
    
    def position_callback(self, msg):
        """
        Update the state based on new position measurements.
        Placeholder function - implement measurement update logic here.
        """
        self.position_msg = msg


if __name__ == "__main__":
    try:
        ose_node = ObjectStateEstimator()
        ose_node.run()
    except rospy.ROSInterruptException:
        pass

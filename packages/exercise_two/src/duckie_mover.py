#!/usr/bin/env python3

import os

import rospy

from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, BoolStamped



class DuckieMover(DTROS):
    def __init__(self, node_name):
        super(DuckieMover, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.vel_pub = rospy.Publisher("/csc22933/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size = 1)
        self.stop_pub = rospy.Publisher("/csc22933/wheels_driver_node/emergency_stop", BoolStamped, queue_size = 1)


    def run(self):
        rate = rospy.Rate(10)
        wait_rate = rospy.Rate(20)

        while self.vel_pub.get_num_connections() < 1:
            wait_rate.sleep()
        while self.stop_pub.get_num_connections() < 1:
            wait_rate.sleep()

        rospy.loginfo("Start...")
        msg_velocity = WheelsCmdStamped()
        for i in range(100):
            msg_velocity.header.stamp = rospy.Time.now()
            msg_velocity.vel_left = 0.50  # https://github.com/duckietown/dt-car-interface/blob/daffy/packages/dagu_car/src/kinematics_node.py
            msg_velocity.vel_right = 0.50
            self.vel_pub.publish(msg_velocity)
            rospy.loginfo("Sending vel")
            rate.sleep()

        for i in range(10):
            msg_velocity.header.stamp = rospy.Time.now()
            msg_velocity.vel_left = 0  # https://github.com/duckietown/dt-car-interface/blob/daffy/packages/dagu_car/src/kinematics_node.py
            msg_velocity.vel_right = 0
            self.vel_pub.publish(msg_velocity)
            rospy.loginfo("Stopping..")
            rate.sleep()
        
        


if __name__ == '__main__':
    # create the node
    node = DuckieMover(node_name='my_mover_node')
    node.run()
    rospy.loginfo("Done")
    # keep spinning
    # rospy.spin()
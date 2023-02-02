#!/usr/bin/env python3

import os
import time

import rospy

from math import pi

from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, WheelEncoderStamped



# class DuckieMover(DTROS):
#     def __init__(self, node_name):
#         super(DuckieMover, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
#         self.vel_pub = rospy.Publisher("/csc22933/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size = 1)
#         self.stop_pub = rospy.Publisher("/csc22933/wheels_driver_node/emergency_stop", BoolStamped, queue_size = 1)


#     def run(self):
#         rate = rospy.Rate(10)
#         wait_rate = rospy.Rate(20)

#         while self.vel_pub.get_num_connections() < 1:
#             wait_rate.sleep()
#         while self.stop_pub.get_num_connections() < 1:
#             wait_rate.sleep()

#         rospy.loginfo("Start...")
#         msg_velocity = WheelsCmdStamped()
#         for i in range(100):
#             msg_velocity.header.stamp = rospy.Time.now()
#             msg_velocity.vel_left = 0.50  # https://github.com/duckietown/dt-car-interface/blob/daffy/packages/dagu_car/src/kinematics_node.py
#             msg_velocity.vel_right = 0.50
#             self.vel_pub.publish(msg_velocity)
#             rospy.loginfo("Sending vel")
#             rate.sleep()

#         for i in range(10):
#             msg_velocity.header.stamp = rospy.Time.now()
#             msg_velocity.vel_left = 0  # https://github.com/duckietown/dt-car-interface/blob/daffy/packages/dagu_car/src/kinematics_node.py
#             msg_velocity.vel_right = 0
#             self.vel_pub.publish(msg_velocity)
#             rospy.loginfo("Stopping..")
#             rate.sleep()


class DuckieMover(DTROS):
    def __init__(self, node_name):
        super(DuckieMover, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self.stop_pub = rospy.Publisher("/"+os.environ['VEHICLE_NAME']+"/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size = 1)
        self.vel_pub = rospy.Publisher("/"+os.environ['VEHICLE_NAME']+"/joy_mapper_node/car_cmd", Twist2DStamped, queue_size = 1)

        self.right_tick_sub = rospy.Subscriber("/"+os.environ['VEHICLE_NAME']+"/right_wheel_encoder_node/tick", 
        WheelEncoderStamped, self.right_tick,  queue_size = 1)
        self.left_tick_sub = rospy.Subscriber("/"+os.environ['VEHICLE_NAME']+"/left_wheel_encoder_node/tick", 
        WheelEncoderStamped, self.left_tick,  queue_size = 1)

        self.rt_val = 0
        self.lt_val = 0


    def go_straight(self, vel, duration):
        msg_velocity = Twist2DStamped()
        msg_velocity.header.stamp = rospy.Time.now()
        msg_velocity.v = vel
        msg_velocity.omega = 0.05
        self.vel_pub.publish(msg_velocity)
        msg_txt = "Sending v: " + str(vel) + ", omega = 0"
        rospy.loginfo(msg_txt)
        time.sleep(duration)

    def stop(self, duration):
        msg_velocity = Twist2DStamped()
        msg_velocity.header.stamp = rospy.Time.now()
        msg_velocity.v = 0
        msg_velocity.omega = 0
        self.vel_pub.publish(msg_velocity)
        msg_txt = "Sending v: 0, omega = 0"
        rospy.loginfo(msg_txt)
        time.sleep(duration)

    def turn(self, omega, duration):
        msg_velocity = Twist2DStamped()
        msg_velocity.header.stamp = rospy.Time.now()
        msg_velocity.v = 0
        msg_velocity.omega = omega
        self.vel_pub.publish(msg_velocity)
        msg_txt = "Sending v: 0, omega = " + str(omega)
        rospy.loginfo(msg_txt)
        time.sleep(duration)

    def right_tick(self, msg):
        self.rt_val = msg.data

    def left_tick(self, msg):
        self.lt_val = msg.data
        

    def run(self):
        rate = rospy.Rate(10)

        while self.stop_pub.get_num_connections() < 1:
            rate.sleep()
        while self.vel_pub.get_num_connections() < 1:
            rate.sleep()

        rospy.loginfo("Start...")

        rospy.loginfo("rt val: " + str(self.rt_val) + ",  lt val: " + str(self.lt_val))

        self.go_straight(vel=0.5, duration=3)
        self.stop(3)

        rospy.loginfo("rt val: " + str(self.rt_val) + ",  lt val: " + str(self.lt_val))
        
        self.turn(pi, 1)
        self.stop(3)

        rospy.loginfo("rt val: " + str(self.rt_val) + ",  lt val: " + str(self.lt_val))

        self.go_straight(vel=0.5, duration=3)
        self.stop(2)

        rospy.loginfo("rt val: " + str(self.rt_val) + ",  lt val: " + str(self.lt_val))       
        

if __name__ == '__main__':
    # create the node
    node = DuckieMover(node_name='my_mover_node')
    node.run()
    rospy.loginfo("Done")
    # keep spinning
    # rospy.spin()
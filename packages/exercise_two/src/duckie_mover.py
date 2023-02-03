#!/usr/bin/env python3

import os
import time

import rospy

from math import pi

from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, WheelEncoderStamped


class DuckieMover(DTROS):
    def __init__(self, node_name):
        super(DuckieMover, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self.stop_pub = rospy.Publisher("/"+os.environ['VEHICLE_NAME']+"/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size = 1)
        self.vel_pub = rospy.Publisher("/"+os.environ['VEHICLE_NAME']+"/joy_mapper_node/car_cmd", Twist2DStamped, queue_size = 1)

        self.right_tick_sub = rospy.Subscriber("/"+os.environ['VEHICLE_NAME']+"/right_wheel_encoder_node/tick", 
        WheelEncoderStamped, self.right_tick,  queue_size = 1)
        self.left_tick_sub = rospy.Subscriber("/"+os.environ['VEHICLE_NAME']+"/left_wheel_encoder_node/tick", 
        WheelEncoderStamped, self.left_tick,  queue_size = 1)

        self.r = rospy.get_param("/"+os.environ['VEHICLE_NAME']+"/kinematics_node/radius", 100)
        rospy.loginfo("Radius of wheel: " +  str(self.r))

        self.reset()

    def reset(self):
        self.rt_val = 0
        self.rt_initial_val = 0
        self.rt_dist = 0

        self.lt_val = 0
        self.lt_initial_val = 0
        self.lt_dist = 0

        self.initial_diff = 0

    def go_straight_dist(self, vel, dist):
        msg_velocity = Twist2DStamped()
        rate = rospy.Rate(20)
        dist_cover = 0
        initial_wheel_diff = 0
        wheel_diff_set = False

        while dist_cover <  abs(dist):
            msg_velocity.header.stamp = rospy.Time.now()
            msg_velocity.v = vel
            omega_offset = 0

            if self.lt_initial_val != 0 and self.rt_initial_val != 0:
                if not wheel_diff_set:
                    wheel_diff_set = True
                    initial_wheel_diff = self.lt_initial_val - self.rt_initial_val

                wheel_diff = self.lt_val - self.rt_val

                if wheel_diff > initial_wheel_diff:
                    omega_offset = wheel_diff / 7500
                else:
                    omega_offset = -wheel_diff / 7500

            msg_velocity.omega = omega_offset
            
            self.vel_pub.publish(msg_velocity)
            rate.sleep()
            dist_cover = abs((self.lt_dist + self.rt_dist)/2)
            # rospy.loginfo("Distance covered: " + str(dist_cover))

        rospy.loginfo("Distance covered: " + str(dist_cover))


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
        # rospy.loginfo(msg.data)
        tk = msg.data + 99999999
        if self.rt_val == 0:
            self.rt_initial_val = tk
        self.rt_val = tk
        tick_diff = self.rt_val - self.rt_initial_val
        self.rt_dist = (2 * pi * self.r * tick_diff) / 135


    def left_tick(self, msg):
        # rospy.loginfo(msg.data)
        tk = msg.data + 99999999
        if self.lt_val == 0:
            self.lt_initial_val = tk
        self.lt_val = tk
        tick_diff = self.lt_val - self.lt_initial_val
        self.lt_dist = (2 * pi * self.r * tick_diff) / 135
        

    def run(self):
        rate = rospy.Rate(10)

        # while self.stop_pub.get_num_connections() < 1:
        #     rate.sleep()
        # while self.vel_pub.get_num_connections() < 1:
        #     rate.sleep()

        rospy.loginfo("Start...")

        self.go_straight_dist(0.5, 1.25)
        self.reset()
        for i in range(10):
            self.stop(0.1)

        self.go_straight_dist(-0.5, 1.25)
        
        for i in range(10):
            self.stop(0.1)

if __name__ == '__main__':
    # create the node
    node = DuckieMover(node_name='my_mover_node')
    node.run()
    rospy.loginfo("Done")
    # keep spinning
    # rospy.spin()
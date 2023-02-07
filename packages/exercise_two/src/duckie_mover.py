#!/usr/bin/env python3

import os
import time

import rospy

from math import pi, cos, sin

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

        self.X = 0.32
        self.Y = 0.32
        self.Th = 0
        self.L = 0.05

        self.reset()

    def reset(self):
        self.rt_initial_set = False
        self.rt = 0
        self.rt_initial_val = 0

        self.lt_initial_set = False
        self.lt = 0
        self.lt_initial_val = 0


    # def go_straight_dist(self, vel, dist):
    #     msg_velocity = Twist2DStamped()
    #     rate = rospy.Rate(30)
    #     dist_cover = 0

    #     while dist_cover <  abs(dist):
    #         msg_velocity.header.stamp = rospy.Time.now()
    #         msg_velocity.v = vel
    #         omega_offset = 0.5
    #         if vel < 0:
    #             omega_offset = 0.1

    #         # if self.lt_initial_val != 0 and self.rt_initial_val != 0:
    #         #     wheel_diff = self.lt_val - self.rt_val

    #             # if wheel_diff > 0:
    #             #     omega_offset = wheel_diff / 7500
    #             # else:
    #             #     omega_offset = -wheel_diff / 7500

    #         msg_velocity.omega = omega_offset
            
    #         self.vel_pub.publish(msg_velocity)
    #         rate.sleep()
            

    #         self.Th  = (self.rt_dist - self.lt_dist) / (2 * self.L)
    #         self.X += (dist_cover * cos(self.Th)) 
    #         self.Y = dist_cover * sin(self.Th)
    #         rospy.loginfo(f'X: {self.X}, Y: {self.Y}, Th: {self.Th}, dist: {dist_cover}')

    #     rospy.loginfo("Distance covered: " + str(dist_cover))


    def stop(self, duration):
        msg_velocity = Twist2DStamped()
        msg_velocity.header.stamp = rospy.Time.now()
        msg_velocity.v = 0
        msg_velocity.omega = 0
        self.vel_pub.publish(msg_velocity)
        # msg_txt = "Sending v: 0, omega = 0"
        # rospy.loginfo(msg_txt)
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
        if not self.rt_initial_set:
            self.rt_initial_set = True
            self.rt_initial_val = msg.data
        self.rt = msg.data - self.rt_initial_val
        # self.rt_dist = (2 * pi * self.r * self.rt_val) / 135


    def left_tick(self, msg):
        if not self.lt_initial_set:
            self.lt_initial_set = True
            self.lt_initial_val = msg.data
        self.lt = msg.data - self.lt_initial_val
        # self.lt_dist = (2 * pi * self.r * self.lt_val) / 135

    def task125cm(self):
        rospy.loginfo("Starting 1.25m task..")

        msg_velocity = Twist2DStamped()
        rate = rospy.Rate(20)
        dist_cover = 0

        vel = 0.5
        prv_rt = 0
        prv_lt = 0

        while dist_cover <  1.25:
            delta_rt = self.rt - prv_rt
            delta_lt = self.lt - prv_lt

            prv_rt = self.rt
            prv_lt = self.lt

            delta_rw_dist = (2 * pi * self.r * delta_rt) / 135
            delta_lw_dist = (2 * pi * self.r * delta_lt) / 135

            delta_dist_cover = (delta_rw_dist + delta_lw_dist)/2
            dist_cover += delta_dist_cover

            self.Th  += (delta_rw_dist - delta_lw_dist) / (2 * self.L)
            self.X += (delta_dist_cover * cos(self.Th)) 
            self.Y += (delta_dist_cover * sin(self.Th))
            
            msg_velocity.header.stamp = rospy.Time.now()
            msg_velocity.v = vel
            msg_velocity.omega = -self.Th * 1.5
            
            self.vel_pub.publish(msg_velocity)
            rospy.loginfo(f'X: {self.X}, Y: {self.Y}, Th: {self.Th}, dist: {dist_cover}, d_rt: {delta_rt}, d_lt: {delta_lt}')
            
            rate.sleep()

        rospy.loginfo("Distance covered: " + str(dist_cover))

        rospy.loginfo("Stopping")
        for i in range(10):
            self.stop(0.1)

        while dist_cover >  0:
            delta_rt = self.rt - prv_rt
            delta_lt = self.lt - prv_lt

            prv_rt = self.rt
            prv_lt = self.lt

            delta_rw_dist = (2 * pi * self.r * delta_rt) / 135
            delta_lw_dist = (2 * pi * self.r * delta_lt) / 135

            delta_dist_cover = (delta_rw_dist + delta_lw_dist)/2
            dist_cover += delta_dist_cover

            self.Th  += (delta_rw_dist - delta_lw_dist) / (2 * self.L)
            self.X += (delta_dist_cover * cos(self.Th)) 
            self.Y += (delta_dist_cover * sin(self.Th))
            
            msg_velocity.header.stamp = rospy.Time.now()
            msg_velocity.v = -vel
            msg_velocity.omega = -self.Th * 1.5
            
            self.vel_pub.publish(msg_velocity)
            rospy.loginfo(f'X: {self.X}, Y: {self.Y}, Th: {self.Th}, dist: {dist_cover}, d_rt: {delta_rt}, d_lt: {delta_lt}')
            
            rate.sleep()
        
        rospy.loginfo("Stopping")
        for i in range(10):
            self.stop(0.1)

        

    def run(self):
        self.task125cm()
  

if __name__ == '__main__':
    # create the node
    node = DuckieMover(node_name='my_mover_node')
    node.run()
    rospy.loginfo("Done")
    # keep spinning
    # rospy.spin()
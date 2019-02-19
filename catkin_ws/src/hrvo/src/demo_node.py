#!/usr/bin/env python

import rospy
from duckiepond_vehicle.msg import UsvDrive
from sensor_msgs.msg import NavSatFix,Imu
from RVO import RVO_update, reach, compute_V_des, reach
import tf

class BoatHRVO(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing" %self.node_name)

        self.pub_v1 = rospy.Publisher("/duck1/cmd_drive",UsvDrive,queue_size=1)
        self.sub_gps1 = rospy.Subscriber("/duck1/fix",NavSatFix,self.cb_gps1,queue_size=1)
        self.sub_imu1 = rospy.Subscriber("/duck1/imu/data",Imu,self.cb_imu1,queue_size=1)
        self.p1 = NavSatFix()
        self.p1.longitude = 121.000261
        self.p1.latitude = 24.788875
        self.y1 = -2.356

        self.pub_v2 = rospy.Publisher("/duck2/cmd_drive",UsvDrive,queue_size=1)
        self.sub_gps2 = rospy.Subscriber("/duck2/fix",NavSatFix,self.cb_gps2,queue_size=1)
        self.sub_imu2 = rospy.Subscriber("/duck2/imu/data",Imu,self.cb_imu2,queue_size=1)
        self.p2 = NavSatFix()
        self.p2.longitude = 121.000115
        self.p2.latitude = 24.788875
        self.y2 = -0.785

        self.pub_v3 = rospy.Publisher("/duck3/cmd_drive",UsvDrive,queue_size=1)
        self.sub_gps3 = rospy.Subscriber("/duck3/fix",NavSatFix,self.cb_gps3,queue_size=1)
        self.sub_imu3 = rospy.Subscriber("/duck3/imu/data",Imu,self.cb_imu3,queue_size=1)
        self.p3 = NavSatFix()
        self.p3.longitude = 121.000115
        self.p3.latitude = 24.788741
        self.y3 = 0.785

        self.pub_v4 = rospy.Publisher("/duck4/cmd_drive",UsvDrive,queue_size=1)
        self.sub_gps4 = rospy.Subscriber("/duck4/fix",NavSatFix,self.cb_gps4,queue_size=1)
        self.sub_imu4 = rospy.Subscriber("/duck4/imu/data",Imu,self.cb_imu4,queue_size=1)
        self.p4 = NavSatFix()
        self.p4.longitude = 121.000261
        self.p4.latitude = 24.788741
        self.y4 = 2.356

        self.ws_model = dict()
        #robot radius
        self.ws_model['robot_radius'] = 0.000015
        self.ws_model['circular_obstacles'] = []
        #rectangular boundary, format [
        #x,y,width/2,heigth/2]
        self.ws_model['boundary'] = [] 

        self.pin1 = [121.000261,24.788875]
        self.pin2 = [121.000115,24.788875]
        self.pin3 = [121.000115,24.788741]
        self.pin4 = [121.000261,24.788741]
        self.position = [self.pin1] + [self.pin2] + [self.pin3] + [self.pin4]
        self.goal = [self.pin3] + [self.pin4] + [self.pin1] + [self.pin2]
        #print(self.position)
        #print(self.goal)
        self.velocity = [[0,0] for i in xrange(len(self.position))]
        self.v_max = [1 for i in xrange(len(self.position))]
        self.R = 1
        self.L = 1

        self.timer = rospy.Timer(rospy.Duration(0.2),self.cb_hrvo)

    def cb_gps1(self,msg):
        self.p1 = msg

    def cb_gps2(self,msg):
        self.p2 = msg

    def cb_gps3(self,msg):
        self.p3 = msg

    def cb_gps4(self,msg):
        self.p4 = msg
    
    def cb_imu1(self,msg):
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.y1 = tf.transformations.euler_from_quaternion(q)[2]

    def cb_imu2(self,msg):
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.y2 = tf.transformations.euler_from_quaternion(q)[2]

    def cb_imu3(self,msg):
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.y3 = tf.transformations.euler_from_quaternion(q)[2]

    def cb_imu4(self,msg):
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.y4 = tf.transformations.euler_from_quaternion(q)[2]
        
    def cb_hrvo(self,event):
        self.position = [[self.p1.longitude,self.p1.latitude]] + [[self.p2.longitude,self.p2.latitude]] + [[self.p3.longitude,self.p3.latitude]] + [[self.p4.longitude,self.p4.latitude]]
        v_des = compute_V_des(self.position,self.goal,self.v_max)
        self.velocity = RVO_update(self.position,v_des,self.velocity,self.ws_model)
        #print(self.position)
        
        self.yaw = [self.y1] + [self.y2] + [self.y3] + [self.y4]
        #print(self.yaw)











if __name__ == "__main__":
    rospy.init_node("BoatHRVO")
    boatHRVO = BoatHRVO()
    rospy.spin()


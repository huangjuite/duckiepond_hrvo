#!/usr/bin/env python

import rospy
from duckiepond_vehicle.msg import UsvDrive
from sensor_msgs.msg import NavSatFix,Imu
from nav_msgs.msg import Odometry
from RVO import RVO_update, reach, compute_V_des, reach
from PID import PID_control
from dynamic_reconfigure.server import Server
from control.cfg import ang_PIDConfig,dis_PIDConfig
import tf

class BoatHRVO(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing" %self.node_name)

        self.dis_pid = [PID_control("distance_control%d" % i) for i in range(4)]
        self.angu_pid = [PID_control("angular_control%d" % i) for i in range(4)]
        self.dis_server = Server(dis_PIDConfig,self.cb_dis_pid,"distance_control")
        self.ang_server = Server(ang_PIDConfig,self.cb_ang_pid,"angular_control")

        self.pub_v1 = rospy.Publisher("/boat1/cmd_drive",UsvDrive,queue_size=1)
        self.sub_p3d1 = rospy.Subscriber("/boat1/p3d_odom",Odometry,self.cb_boat1_odom,queue_size=1)

        self.pub_v2 = rospy.Publisher("/boat2/cmd_drive",UsvDrive,queue_size=1)
        self.sub_p3d1 = rospy.Subscriber("/boat2/p3d_odom",Odometry,self.cb_boat2_odom,queue_size=1)

        self.pub_v3 = rospy.Publisher("/boat3/cmd_drive",UsvDrive,queue_size=1)
        self.sub_p3d1 = rospy.Subscriber("/boat3/p3d_odom",Odometry,self.cb_boat3_odom,queue_size=1)

        self.pub_v4 = rospy.Publisher("/boat4/cmd_drive",UsvDrive,queue_size=1)
        self.sub_p3d1 = rospy.Subscriber("/boat4/p3d_odom",Odometry,self.cb_boat4_odom,queue_size=1)

        self.boat_odom = []
        for i in range(4):
            self.boat_odom.append(Odometry())

        self.yaw = [[0] for i in range(4)]

        self.ws_model = dict()
        #robot radius
        self.ws_model['robot_radius'] = 1.5
        self.ws_model['circular_obstacles'] = []
        #rectangular boundary, format [
        #x,y,width/2,heigth/2]
        self.ws_model['boundary'] = [] 

        self.pin1 = [7.5,7.5]
        self.pin2 = [-7.5,7.5]
        self.pin3 = [-7.5,-7.5]
        self.pin4 = [7.5,-7.5]
        self.position = [self.pin1] + [self.pin2] + [self.pin3] + [self.pin4]
        self.goal = [self.pin3] + [self.pin4] + [self.pin1] + [self.pin2]
        #print(self.position)
        #print(self.goal)
        self.velocity = [[0,0] for i in xrange(len(self.position))]
        self.v_max = [1 for i in xrange(len(self.position))]
        self.R = 1
        self.L = 1

        self.timer = rospy.Timer(rospy.Duration(0.2),self.cb_hrvo)


    def cb_hrvo(self,event):
        self.pos_update()
        v_des = compute_V_des(self.position,self.goal,self.v_max)
        self.velocity = RVO_update(self.position,v_des,self.velocity,self.ws_model)
        #print("position",self.position)
        #print("velocity",self.velocity)

    def pos_update(self):
        self.position = []
        for i in range(4):
            pos = [self.boat_odom[i].pose.pose.position.x,self.boat_odom[i].pose.pose.position.y]
            self.position.append(pos)
            quaternion = (self.boat_odom[i].pose.pose.orientation.x,
                        self.boat_odom[i].pose.pose.orientation.y,
                        self.boat_odom[i].pose.pose.orientation.z,
                        self.boat_odom[i].pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.yaw[i] = euler[2]

    def cb_boat1_odom(self,msg):
        self.boat_odom[0] = msg

    def cb_boat2_odom(self,msg):
        self.boat_odom[1] = msg
    
    def cb_boat3_odom(self,msg):
        self.boat_odom[2] = msg

    def cb_boat4_odom(self,msg):
        self.boat_odom[3] = msg

    def cb_dis_pid(self,config,level):
        print("distance: [Kp]: {Kp}   [Ki]: {Ki}   [Kd]: {Kd}\n".format(**config))
        Kp = float("{Kp}".format(**config))
        Ki = float("{Ki}".format(**config))
        Kd = float("{Kd}".format(**config))
        for i in range(4):
            self.dis_pid[i].setKp(Kp)
            self.dis_pid[i].setKi(Ki)
            self.dis_pid[i].setKd(Kd)
        return config

    def cb_ang_pid(self,config,level):
        print("angular: [Kp]: {Kp}   [Ki]: {Ki}   [Kd]: {Kd}\n".format(**config))
        Kp = float("{Kp}".format(**config))
        Ki = float("{Ki}".format(**config))
        Kd = float("{Kd}".format(**config))
        for i in range(4):
            self.angu_pid[i].setKp(Kp)
            self.angu_pid[i].setKi(Ki)
            self.angu_pid[i].setKd(Kd)
        return config


if __name__ == "__main__":
    rospy.init_node("BoatHRVO")
    boatHRVO = BoatHRVO()
    rospy.spin()


#! /usr/bin/env python

import rospy
import sys
import copy

import tf2_ros
import tf2_msgs.msg
import moveit_commander
from hrwros_gazebo.msg  import LogicalCameraImage
from pkg_vb_sim.srv import *

class Logical_camera:
    def __init__(self):

        self.m=[]
        rospy.init_node('node_eg5_waypoints', anonymous=True)
        rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, self.callback)

    def callback(self,data):
        '''This is a callback function for logical camera '''
        self.m=data.models

    def conveyor_belt(self,power):
        ''' this is a function for conveyor belt '''
        rospy.wait_for_service("/eyrc/vb/conveyor/set_power")
        conve_yor = rospy.ServiceProxy("/eyrc/vb/conveyor/set_power", conveyorBeltPowerMsg)
        conveyor= conve_yor(power)
        return conveyor

def main():


    ur5 = Logical_camera()
    print(ur5.m)
    if len(ur5.m) == 0:
        rospy.loginfo("i am in if condition")
        ur5.conveyor_belt(57)
        print(ur5.m)
    if len(ur5.m) >0:
        rospy.sleep(1.25)
        rospy.loginfo("i am in else condition")
        ur5.conveyor_belt(0)
        rospy.sleep(6.5)
        print(ur5.m)




if __name__ == '__main__':
    rospy.sleep(30)
    while not rospy.is_shutdown():
        main()

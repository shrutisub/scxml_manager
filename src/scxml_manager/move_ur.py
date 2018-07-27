#!/usr/bin/env python

import rospy
import smach
import moveit_commander
import sys
import os
from sensor_msgs.msg import JointState
import ast
import numpy as np

class Movearticular(smach.State):
    def __init__(self,outcomes=["preempt","success"], io_keys=["group","position","joint_name","vel_factor","type"]):
        smach.State.__init__(self,outcomes,io_keys=io_keys )
        self.group = None

    def check_angle_type(self,type):
        target=[]
        for element in target:
            if(type=="degree"):
                target[target.index(element)]=np.deg2rad(element)#conversion from deg 2 radian
            elif(type=="radian"):
                float_elementes=float(element)
                target[target.index(element)]=float_elements
    def execute(self, ud):
        target=[]
        target_values=[]
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        rospy.sleep(5)
        self.group.set_max_velocity_scaling_factor(float(ud.vel_factor))
        self.group.set_start_state_to_current_state()
        #target of the joints can be dict,list or string and thus we have to check the instances passed as all of these target types
        if isinstance(ud.position,list) and ud.joint_name is None:
            target_list = [int(x) for x in ud.position]  #conversion if each element is provided in string format
            for element in target_list:
                target.append(element)
                self.check_angle_type(ud.type)
        elif isinstance(ud.position,dict):
            dict_iter=iter(sorted(ud.position.items()))
            for key,values in dict_iter:
                target_values.append(values)
            target_list = [int(x) for x in target_values]
            for element in target_list:
                target.append(element)
                seld.check_angle_type(ud.type)
        elif isinstance(ud.position,list) and ud.joint_name is not None:
            js= JointState()
            js.header.stamp = rospy.Time.now()
            js.name=ud.joint_name
            js.position=ud.position #the type should be float
            for element in js.position:
                target.append(element)
                self.check_angle_type(ud.type)
        try:
                self.group.set_joint_value_target(target)
                motionplan=self.group.plan()
                self.group.execute(motionplan) #executing the motion motionplan
                #self.group.go()
                rospy.sleep(1)
        except Exception as ex:
                rospy.logerr(ex)
                return "preempt"
        return "success"

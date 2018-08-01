#!/usr/bin/env python

import rospy
import smach
import moveit_commander
import sys
import os
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi,radians
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import ast
import numpy as np
from copy import deepcopy



'''
class Moveskill(smach.State):
    def __init__(self,outcomes=["preempt","success"], io_keys=["id"]):
        smach.State.__init__(self,outcomes,io_keys=io_keys )

    def execute(self,ud):
        if(ud.id==1):
            return "success"
        else:
            return "preempt"
'''
class Choice(smach.State):
    def __init__(self,outcomes=["Transition_3","Transition_4","preempt"], io_keys=["movement"]):
        smach.State.__init__(self,outcomes,io_keys=io_keys )

    def execute(self,ud):
        if(ud.movement=="Movearticular"):
            ma=Movearticular()
            print " i m here"
            return "Transition_3"
        elif(ud.movement=="Movecartesian"):
            mc=Movecartesian()
            print "i m in cartesian state"
            return "Transition_4"
        else:
            return "preempt"

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

class Movecartesian(smach.State):

    def __init__(self,outcomes=["preempt","success"], io_keys=["group","target"]):
        smach.State.__init__(self,outcomes,io_keys=io_keys )
        self.group=None

    def execute(self, ud):
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        endeffector=self.group.get_end_effector_link()
        Referencelink="/base_link"
        self.group.set_pose_reference_frame(Referencelink)
        self.group.allow_replanning(True)
        self.group.set_goal_position_tolerance(0.01)
        self.group.set_goal_orientation_tolerance(0.1)
        start_pose = self.group.get_current_pose(endeffector).pose
        waypoints = []
        waypoints.append(start_pose)
        # first orient gripper and move forward (+x)
        wpose = deepcopy(start_pose)
        wpose.position.z += 0.74
        waypoints.append(deepcopy(wpose))
        fraction = 0.0
        maximum_points = 10
        counter = 0
        # Set the internal state to the current state
        self.group.set_start_state_to_current_state()
        # Plan the Cartesian path connecting the waypoints
        while fraction < 1.0 and counter < maximum_points:
            (plan, fraction) = self.group.compute_cartesian_path (waypoints, 0.01, 0.0, True)
            # Increment the number of counter
            counter += 1
            # Print out a progress message
            if counter % 10 == 0:
                rospy.loginfo("Still trying after " + str(counter) + " counter...")

        # If we have a complete plan, execute the trajectory
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            self.group.execute(plan)
            rospy.sleep(1)
            rospy.loginfo("Path execution complete.")
            return "success"
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " counter.")
            return "preempt"

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
from collections import OrderedDict
import ast
import numpy as np
from copy import deepcopy


class InitMoveGroup(smach.State):
        def __init__(self,outcomes=["success","preempt"],io_keys=["group"]):
            smach.State.__init__(self,outcomes,io_keys=io_keys)
        def execute(self,ud):
            if ud.group=='denso_robot':
                print "i m here"
                return "success"
            else:
                return "preempt"

class GetJS(smach.State):
        def __init__(self,outcomes=["success","preempt"]):
            smach.State.__init__(self,outcomes)

        def execute(self,ud):
            angles=dict()
            try:
                msg = rospy.wait_for_message("/joint_states", JointState,timeout=2)
            except rospy.exceptions.ROSInterruptException:
                return "preempt"
            angles=Movearticular().get_joints(msg.name)
            return angles

class Movearticular(smach.State):
        def __init__(self,outcomes=["preempt","success"], io_keys=["group","position_js","joint_name","vel_factor","type"]):
            smach.State.__init__(self,outcomes,io_keys=io_keys )
            self.group = None

        def set_joints(self, target_js):
            list_position=[]
            ma_pos={}
            try:
                msg = rospy.wait_for_message("/joint_states", JointState,timeout=2)
            except rospy.exceptions.ROSInterruptException:
                exit(0)
            ma_pos = [0.0 for x in range(len(msg.position))]
            for id, name in enumerate(msg.name):
                if name in target_js.iterkeys():
                    ma_pos[id] = target_js[name]
                else:
                    ma_pos[id] = msg.position[id]
            return ma_pos

        def check_angle_type(self,type,target):
            for element in target:
                if(type=="degree"):
                    target[target.index(element)]=np.deg2rad(element)#conversion from deg 2 radian
                elif(type=="radian"):
                    float_elements=float(element)
                    target[target.index(element)]=float_elements
            return target

        def execute(self, ud):
            target=[]
            target_values=[]
            target_joints=[]
            self.group = moveit_commander.MoveGroupCommander("denso_robot")
            rospy.sleep(5)
            self.group.set_max_velocity_scaling_factor(float(ud.vel_factor))
            self.group.set_start_state_to_current_state()
            #target of the joints can be dict,list or string and thus we have to check the instances passed as all of these target types
            if isinstance(ud.position_js,list):
                target=self.check_angle_type(ud.type,ud.position_js)
            elif isinstance(ud.position_js,dict):
                ma_position = self.set_joints(ud.position_js)
                print("The position %s"%ma_position)
                target=self.check_angle_type(ud.type, ma_position)
                print("target values", target)
            elif isinstance(ud.position_js,JointState):#TODO #isinstance(ud.position_js,GetJS)
                #position=self.get_joints(ud.joint_name,ud.position_js)
                target=self.check_angle_type(ud.type,ud.position_js)
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

        def __init__(self,outcomes=["preempt","success"], io_keys=["target"]):
            smach.State.__init__(self,outcomes,io_keys=io_keys )
            self.group=None

        def convertToPose(self, pt):
            if(isinstance(pt,Pose)):
                return pt
            else:
                pose = Pose()
                if(len(pt) != 7):
                    rospy.logerr("[MoveCartesian] Not well formated point in the userdata 'target' : %s "%str(pt))
                    raise Exception()
                else:
                    pose.position.x = pt[0]
                    pose.position.y = pt[1]
                    pose.position.z = pt[2]
                    pose.orientation.x = pt[3]
                    pose.orientation.y = pt[4]
                    pose.orientation.z = pt[5]
                    pose.orientation.w = pt[6]
                return pose
        def execute(self, ud):
            print "Cartesian path"
            self.group = moveit_commander.MoveGroupCommander("denso_robot")
            rospy.sleep(5)
            Referencelink="/base_link"
            self.group.set_pose_reference_frame(Referencelink)
            self.group.allow_replanning(True)
            self.group.set_goal_position_tolerance(0.01)
            self.group.set_goal_orientation_tolerance(0.1)
            endeffector=self.group.get_end_effector_link()
            waypoints = []
            if(isinstance(ud.target,list)):
                if(isinstance(ud.target[0],list)): ##multiple points
                   for pt in ud.target:
                       print "I am here"
                       waypoints.append(self.convertToPose(pt))
                else: ##only one point
                    print "I am here in else"
                    waypoints.append(self.convertToPose(ud.target))
            fraction = 0.0
            # Set the internal state to the current state
            self.group.set_start_state_to_current_state()
            # Plan the Cartesian path connecting the waypoints
            while fraction < 1.0:
                (plan, fraction) = self.group.compute_cartesian_path (waypoints, 0.01, 0.0, True)
                print fraction
            # If we have a complete plan, execute the trajectory
            if fraction==1.0:
                rospy.loginfo("Path computed successfully. Moving the arm.")
                self.group.execute(plan)
                rospy.sleep(5)

                rospy.loginfo("Path execution complete.")
                return "success"
            else:
                rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maximum_points) + " counter.")
                return "preempt"

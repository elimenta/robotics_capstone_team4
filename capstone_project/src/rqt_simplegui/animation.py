#!/usr/bin/env python
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('trajectory_msgs')
roslib.load_manifest('control_msgs')
roslib.load_manifest('pr2_mechanism_msgs')
roslib.load_manifest('sensor_msgs')
roslib.load_manifest('actionlib')

import rospy
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryGoal, JointTrajectoryAction
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from subprocess import call
from pr2_mechanism_msgs.srv import SwitchController


class AnimationPlayer:
    def __init__(self, left_poses, right_poses):
        # Sequence of poses to be used in our animation (assumes both are the same size for now)
        # Note that poses refers to an array of arrays of 7 coordinates for joints
        
        self.left_poses  = left_poses
        self.right_poses = right_poses
        self.r_joint_names = ['r_shoulder_pan_joint',
                              'r_shoulder_lift_joint',
                              'r_upper_arm_roll_joint',
                              'r_elbow_flex_joint',
                              'r_forearm_roll_joint',
                              'r_wrist_flex_joint',
                              'r_wrist_roll_joint']
                              
        self.l_joint_names = ['l_shoulder_pan_joint',
                              'l_shoulder_lift_joint',
                              'l_upper_arm_roll_joint',
                              'l_elbow_flex_joint',
                              'l_forearm_roll_joint',
                              'l_wrist_flex_joint',
                              'l_wrist_roll_joint']
        
        # For relaxing/ freezeing arms
        switch_srv_name = 'pr2_controller_manager/switch_controller'
        rospy.loginfo('Waiting for switch controller service...')
        rospy.wait_for_service(switch_srv_name)
        self.switch_service_client = rospy.ServiceProxy(switch_srv_name, SwitchController)
        
        # Create trajectory action client for arm movements
        r_traj_controller_name = '/r_arm_controller/joint_trajectory_action'
        self.r_traj_action_client = SimpleActionClient(r_traj_controller_name, JointTrajectoryAction)
        rospy.loginfo('Waiting for a response from the trajectory action server for RIGHT arm...')
        self.r_traj_action_client.wait_for_server()
        
        l_traj_controller_name = '/l_arm_controller/joint_trajectory_action'
        self.l_traj_action_client = SimpleActionClient(l_traj_controller_name, JointTrajectoryAction)
        rospy.loginfo('Waiting for a response from the trajectory action server for LEFT arm...')
        self.l_traj_action_client.wait_for_server()
        
    def play(self, duration):
        for i in range(len(self.left_poses)):
            self.move_arm('l', self.left_poses[i], self.right_poses[i], duration)
            self.move_arm('r', self.left_poses[i], self.right_poses[i], duration)
            self.l_traj_action_client.wait_for_result()
            self.r_traj_action_client.wait_for_result()
        
    def move_arm(self, side_prefix, left_pose, right_pose, duration):
        if (side_prefix == 'r'):
            if right_pose == None:
                rospy.logerr('Target pose does not exist.')
            else:
                self.toggle_arm('r', 'Freeze', False)
                self.move_to_joints('r', right_pose, float(duration))
        elif (side_prefix == 'l'):
            if left_pose == None:
                rospy.logerr('Target pose does not exist.')
            else:
                self.toggle_arm('l', 'Freeze', False)
                self.move_to_joints('l', left_pose, float(duration))

        # Move both arms according to the selected boxes on the GUI
        else:
            if right_pose != None:
                self.toggle_arm('r', 'Freeze', False)
                self.move_to_joints('r', right_pose, float(duration))
            if left_pose != None:
                self.toggle_arm('l', 'Freeze', False)
                self.move_to_joints('l', left_pose, float(duration))

    def move_to_joints(self, side_prefix, positions, time_to_joint):
        '''Moves the arm to the desired joints'''
        velocities = [0] * len(positions)
        traj_goal = JointTrajectoryGoal()
        traj_goal.trajectory.header.stamp = (rospy.Time.now() + rospy.Duration(0.1))
        traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=positions,
                            velocities=velocities, time_from_start=rospy.Duration(time_to_joint)))
    
        if (side_prefix == 'r'):
            traj_goal.trajectory.joint_names = self.r_joint_names
            self.r_traj_action_client.send_goal(traj_goal)
        else:
            traj_goal.trajectory.joint_names = self.l_joint_names
            self.l_traj_action_client.send_goal(traj_goal)
            
    def toggle_arm(self, side, toggle, button):
        controller_name = side + '_arm_controller'
        
        start_controllers = []
        stop_controllers = []
        if (toggle == 'Relax'):
            stop_controllers.append(controller_name)
        else:
            start_controllers.append(controller_name)
        self.set_arm_mode(start_controllers, stop_controllers)

    def set_arm_mode(self, start_controllers, stop_controllers):
        try:
            self.switch_service_client(start_controllers, stop_controllers, 1)
        except rospy.ServiceException:
            rospy.logerr('Could not change arm mode.')

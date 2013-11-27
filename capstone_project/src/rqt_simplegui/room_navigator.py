#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('actionlib_msgs')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('move_base_msgs')

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt
from tf import TransformListener

class RoomNavigator():
    def __init__(self):
        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 3)
        
        # Are we running in the fake simulator?
        self.fake_test = rospy.get_param("~fake_test", False)
        
        # Goal state return values
        self.goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']
        
        # Set up the goal locations. Poses are defined in the map frame.
        # An easy way to find the pose coordinates is to point-and-click
        # Nav Goals in RViz when running in the simulator.
        # Pose coordinates are then displayed in the terminal
        # that was used to launch RViz.
        self.locations = dict()

        self.locations['trash'] = Pose(Point(-1.20214903355, -0.475028038025, 0.00), Quaternion(0, 0.000,  0.984054755563, -0.177865786634))
        
        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        
        rospy.loginfo("Connected to move base server")
        
        # A variable to hold the initial pose of the robot to be set by
        # the user in RViz
        self.initial_pose = PoseWithCovarianceStamped()

        # A transform listener to get the current position of the robot in the map
        # self.tf_listener = TransformListener()
        
        # Get the initial pose from the user
        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        self.last_location = Pose()
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
        
    def move_to_bin(self):
        #Set up the next goal location
        self.goal = MoveBaseGoal()
        self.goal.target_pose.pose = self.locations['trash']
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        
        # Let the user know where the robot is going next
        rospy.loginfo("Going to trash")
        
        # Start the robot toward the next location
        self.move_base.send_goal(self.goal)
        
        # Allow 1 minute to get there
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))
        
        # Check for success or failure
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
                rospy.loginfo("State:" + str(state))
            else:
              rospy.loginfo("Goal failed with error code: " + str(self.goal_states[state]))

    def move_to_trash_location(self, target_pose):

    	 #Set up the next goal location
        self.goal = MoveBaseGoal()
        self.goal.target_pose.pose = target_pose
        self.goal.target_pose.pose.position.z = 0
        self.goal.target_pose.pose.orientation.x = 0
        self.goal.target_pose.pose.orientation.y = 0
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        
        # Let the user know where the robot is going next
        rospy.loginfo("Moving towards trash")
        
        # Start the robot toward the next location
        self.move_base.send_goal(self.goal)

         # Allow 1 minute to get there
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))
        
        # Check for success or failure
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
                rospy.loginfo("State:" + str(state))
            else:
              rospy.loginfo("Goal failed with error code: " + str(self.goal_states[state]))
    
    def get_distance_from_target(self, target_pose):
    	# Get the latest available transform by passing time 0
    	try:
    		self.tf_listener.waitForTransform('/map', '/base_link', rospy.Time())
    		(trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    		rospy.logwarn('TF exception during transform')
    		return
    	rospy.loginfo("Current position: " + str(trans))
    	return sqrt(pow(trans[0] - target_pose.position.x, 2) + pow(pow(trans[1] - target_pose.position.y, 2)))

    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
  
    def trunc(f, n):
        # Truncates/pads a float f to n decimal places without rounding
        slen = len('%.*f' % (n, f))
        return float(str(f)[:slen])
            
        

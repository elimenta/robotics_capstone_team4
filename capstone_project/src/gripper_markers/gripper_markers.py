#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('actionlib')
roslib.load_manifest('actionlib_msgs')
roslib.load_manifest('control_msgs') #
roslib.load_manifest('trajectory_msgs') #
roslib.load_manifest('sensor_msgs') #
roslib.load_manifest('pr2_mechanism_msgs') #
roslib.load_manifest('interactive_markers')
roslib.load_manifest('tf')

import numpy
import rospy
import tf
from tf import TransformListener
from geometry_msgs.msg import Quaternion, Vector3, Point, Pose
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from ik import IK
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryGoal, JointTrajectoryAction
from pr2_mechanism_msgs.srv import SwitchController
from sensor_msgs.msg import JointState
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus


class GripperMarkers:

    _offset = 0.09
    def __init__(self, side_prefix):
        self.ik = IK(side_prefix)
	
        self.side_prefix = side_prefix
        self._im_server = InteractiveMarkerServer('ik_request_markers')
        self._tf_listener = TransformListener()
        self._menu_handler = MenuHandler()

        self._menu_handler.insert('Move arm here', callback=self.move_to_pose_cb)

        self.is_control_visible = False
        self.ee_pose = self.get_ee_pose()
        self.update_viz()
        self._menu_handler.apply(self._im_server, 'ik_target_marker')
        self._im_server.applyChanges()
        
        # Code for moving the robots joints
        switch_srv_name = 'pr2_controller_manager/switch_controller'
        rospy.loginfo('Waiting for switch controller service...')
        rospy.wait_for_service(switch_srv_name)
        self.switch_service_client = rospy.ServiceProxy(switch_srv_name,
                                                 SwitchController)
                                                 
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
                              
        # Create a trajectory action client
        r_traj_controller_name = '/r_arm_controller/joint_trajectory_action'
        self.r_traj_action_client = SimpleActionClient(r_traj_controller_name, JointTrajectoryAction)
        rospy.loginfo('Waiting for a response from the trajectory action server for RIGHT arm...')
        self.r_traj_action_client.wait_for_server()
        
        l_traj_controller_name = '/l_arm_controller/joint_trajectory_action'
        self.l_traj_action_client = SimpleActionClient(l_traj_controller_name, JointTrajectoryAction)
        rospy.loginfo('Waiting for a response from the trajectory action server for LEFT arm...')
        self.l_traj_action_client.wait_for_server()

    def get_ee_pose(self):
        from_frame = 'base_link'
        to_frame = self.side_prefix + '_wrist_roll_link'
        try:
           
            t = self._tf_listener.getLatestCommonTime(from_frame, to_frame)
 
            (pos, rot) = self._tf_listener.lookupTransform(from_frame, to_frame, t)

        except:

            rospy.logerr('Could not get end effector pose through TF.')
            pos = [1.0, 0.0, 1.0]
            rot = [0.0, 0.0, 0.0, 1.0]

        
        return Pose(Point(pos[0], pos[1], pos[2]),
               Quaternion(rot[0], rot[1], rot[2], rot[3]))

    def move_to_pose_cb(self, dummy):
        rospy.loginfo('You pressed the `Move arm here` button from the menu.')
        joint_positions = self.ik.get_ik_for_ee(self.ee_pose)
        print 'joint: ' + str(joint_positions)
        if(joint_positions != None):
            self.toggle_arm(self.side_prefix, 'Freeze', False)
            #joint_positions = [-0.993560463247, -0.357041076593, 0.0534981848008, -1.24934444608, -3.10068096283, -1.7906747183, -28.4704855309]
            print 'returned from IK: ' + str(joint_positions)
            #print 'random position : ' + str([-0.993560463247, -0.357041076593, 0.0534981848008, -1.24934444608, -3.10068096283, -1.7906747183, -28.4704855309])
            #joint_positions = [-1.03129153, -0.35312086, -0.08, -1.25402906, -2.98718287, -1.7816027, 3.03965124]
            self.move_to_joints(self.side_prefix, list(joint_positions), 5.0)
        print 'done'
        
    def toggle_arm(self, side, toggle, button):
        controller_name = side + '_arm_controller'
        print 'toggle'
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

    def marker_clicked_cb(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.ee_pose = feedback.pose
            self.update_viz()
            self._menu_handler.reApply(self._im_server)
            self._im_server.applyChanges()

        elif feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.loginfo('Changing visibility of the pose controls.')
            if (self.is_control_visible):
                self.is_control_visible = False
            else:
                self.is_control_visible = True
        else:
            rospy.loginfo('Unhandled event: ' + str(feedback.event_type))

    def update_viz(self):
        
        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
        menu_control.always_visible = True
        frame_id = 'base_link'
        pose = self.ee_pose
        
        menu_control = self._add_gripper_marker(menu_control)
        text_pos = Point()
        text_pos.x = pose.position.x
        text_pos.y = pose.position.y
        text_pos.z = pose.position.z + 0.1
        text = 'x=' + str(pose.position.x) + ' y=' + str(pose.position.y) + ' x=' + str(pose.position.z)
        menu_control.markers.append(Marker(type=Marker.TEXT_VIEW_FACING,
                                           id=0, scale=Vector3(0, 0, 0.03),
                                           text=text,
                                           color=ColorRGBA(0.0, 0.0, 0.0, 0.5),
                                           header=Header(frame_id=frame_id),
                                           pose=Pose(text_pos, Quaternion(0, 0, 0, 1))))
        int_marker = InteractiveMarker()
        int_marker.name = 'ik_target_marker'
        int_marker.header.frame_id = frame_id
        int_marker.pose = pose
        int_marker.scale = 0.2
        self._add_6dof_marker(int_marker)
        int_marker.controls.append(menu_control)
        self._im_server.insert(int_marker, self.marker_clicked_cb)

    def _add_gripper_marker(self, control):
        is_hand_open=False
        if is_hand_open:
            angle = 28 * numpy.pi / 180.0
        else:
            angle = 0
        transform1 = tf.transformations.euler_matrix(0, 0, angle)
        transform1[:3, 3] = [0.07691 - GripperMarkers._offset, 0.01, 0]
        transform2 = tf.transformations.euler_matrix(0, 0, -angle)
        transform2[:3, 3] = [0.09137, 0.00495, 0]
        t_proximal = transform1
        t_distal = tf.transformations.concatenate_matrices(transform1, transform2)
        mesh1 = self._make_mesh_marker()
        mesh1.mesh_resource = ('package://pr2_description/meshes/gripper_v0/gripper_palm.dae')
        mesh1.pose.position.x = -GripperMarkers._offset
        mesh1.pose.orientation.w = 1
        mesh2 = self._make_mesh_marker()
        mesh2.mesh_resource = ('package://pr2_description/meshes/gripper_v0/l_finger.dae')
        mesh2.pose = GripperMarkers.get_pose_from_transform(t_proximal)
        mesh3 = self._make_mesh_marker()
        mesh3.mesh_resource = ('package://pr2_description/meshes/gripper_v0/l_finger_tip.dae')
        mesh3.pose = GripperMarkers.get_pose_from_transform(t_distal)
        quat = tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_from_euler(numpy.pi, 0, 0),
        tf.transformations.quaternion_from_euler(0, 0, angle))
        transform1 = tf.transformations.quaternion_matrix(quat)
        transform1[:3, 3] = [0.07691 - GripperMarkers._offset, -0.01, 0]
        transform2 = tf.transformations.euler_matrix(0, 0, -angle)
        transform2[:3, 3] = [0.09137, 0.00495, 0]
        t_proximal = transform1
        t_distal = tf.transformations.concatenate_matrices(transform1,transform2)
        mesh4 = self._make_mesh_marker()
        mesh4.mesh_resource = ('package://pr2_description/meshes/gripper_v0/l_finger.dae')
        mesh4.pose = GripperMarkers.get_pose_from_transform(t_proximal)
        mesh5 = self._make_mesh_marker()
        mesh5.mesh_resource = ('package://pr2_description/meshes/gripper_v0/l_finger_tip.dae')
        mesh5.pose = GripperMarkers.get_pose_from_transform(t_distal)

        control.markers.append(mesh1)
        control.markers.append(mesh2)
        control.markers.append(mesh3)
        control.markers.append(mesh4)
        control.markers.append(mesh5)
        return control

    @staticmethod
    def get_pose_from_transform(transform):
        pos = transform[:3,3].copy()
        rot = tf.transformations.quaternion_from_matrix(transform)
        return Pose(Point(pos[0], pos[1], pos[2]),
                    Quaternion(rot[0], rot[1], rot[2], rot[3]))

    def _make_mesh_marker(self):
        mesh = Marker()
        mesh.mesh_use_embedded_materials = False
        mesh.type = Marker.MESH_RESOURCE
    	mesh.scale.x = 1.0
        mesh.scale.y = 1.0
        mesh.scale.z = 1.0
        mesh.color = ColorRGBA(0.0, 0.5, 1.0, 0.6)
        ik_solution = self.ik.get_ik_for_ee(self.ee_pose)
        if (ik_solution == None):
            mesh.color = ColorRGBA(1.0, 0.0, 0.0, 0.6)
        else:
            mesh.color = ColorRGBA(0.0, 1.0, 0.0, 0.6)
        return mesh

    def _add_6dof_marker(self, int_marker):
        is_fixed = True
        control = self._make_6dof_control('rotate_x', Quaternion(1, 0, 0, 1), False, is_fixed)
        int_marker.controls.append(control)
        control = self._make_6dof_control('move_x', Quaternion(1, 0, 0, 1), True, is_fixed)
        int_marker.controls.append(control)
        control = self._make_6dof_control('rotate_z', Quaternion(0, 1, 0, 1), False, is_fixed)
        int_marker.controls.append(control)
        control = self._make_6dof_control('move_z', Quaternion(0, 1, 0, 1), True, is_fixed)
        int_marker.controls.append(control)
        control = self._make_6dof_control('rotate_y', Quaternion(0, 0, 1, 1), False, is_fixed)
        int_marker.controls.append(control)
        control = self._make_6dof_control('move_y', Quaternion(0, 0, 1, 1), True, is_fixed)
        int_marker.controls.append(control)

    def _make_6dof_control(self, name, orientation, is_move, is_fixed):
        control = InteractiveMarkerControl()
        control.name = name
        control.orientation = orientation
        control.always_visible = False
        if (self.is_control_visible):
            if is_move:
                control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            else:
                control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        else:
            control.interaction_mode = InteractiveMarkerControl.NONE
        if is_fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        return control

if __name__=='__main__':
    rospy.init_node('ik_target_marker_server')
    r_gm = GripperMarkers('r')
    rospy.spin()



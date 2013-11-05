#!/usr/bin/env python

import roslib
roslib.load_manifest('sound_play')
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('actionlib_msgs')
roslib.load_manifest('control_msgs')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('visualization_msgs') #lab 6

from subprocess import call
import re
import os
import fileinput #for directory reading
import rospy
import threading
from qt_gui.plugin import Plugin
from python_qt_binding import QtGui, QtCore
from python_qt_binding.QtGui import QWidget, QFrame, QGroupBox, QListWidget, QListWidgetItem, QAbstractItemView, QMessageBox
from python_qt_binding.QtCore import QSignalMapper, qWarning, Signal
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from control_msgs.msg import GripperCommandAction, GripperCommandGoal, PointHeadAction, PointHeadGoal
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, Vector3, Point, Quaternion, Pose
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryGoal, JointTrajectoryAction
from pr2_mechanism_msgs.srv import SwitchController
from sensor_msgs.msg import JointState
from animation import AnimationPlayer
from pair import Pair

class SimpleGUI(Plugin):
    
    # For sending speech
    sound_sig = Signal(SoundRequest)
    
    # Joints for arm poses
    joint_sig = Signal(JointState)
    

    def __init__(self, context):
        self.prompt_width = 170
        self.input_width = 250    
        
        super(SimpleGUI, self).__init__(context)
        self.setObjectName('SimpleGUI')
        self._widget = QWidget()     

        self._sound_client = SoundClient()
        
        #find relative path for files to load
        self.local_dir = os.path.dirname(__file__)
        self.dir = os.path.join(self.local_dir, './lib/rqt_simplegui/')
        if not os.path.isdir(self.dir):
            os.makedirs(self.dir)
        
        #need to add any additional subfolders as needed
        if not os.path.isdir(self.dir + 'animations/'):
            os.makedirs(self.dir + 'animations/')
        
 
        # Creates a subscriber to the ROS topic, having msg type SoundRequest 
        rospy.Subscriber('robotsound', SoundRequest, self.sound_cb)

        QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))
        self.sound_sig.connect(self.sound_sig_cb)
        
        # Code used for saving/ loading arm poses for the robot
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

        self.all_joint_names = []
        self.all_joint_poses = []

        # Hash tables storing the name of the pose and the
        # associated positions for that pose, initially empty
        self.saved_l_poses = {}
        self.saved_r_poses = {}
        
        # Hash tables for storing name of animations and the associated pose list
        self.saved_animations = {}
        
        '''l_path = '/home/team4/catkin_ws/lib/rqt_simplegui/l_arm_poses/'
        l_listing = os.listdir(l_path)
        for infile in l_listing:
            positions = []
            for line in fileinput.input(l_path + infile):
                positions.append(float(line))
            self.saved_l_poses[os.path.splitext(infile)[0]] = positions
        
        r_path = '/home/team4/catkin_ws/lib/rqt_simplegui/r_arm_poses/'
        r_listing = os.listdir(r_path)
        for infile in r_listing:
            positions = []
            for line in fileinput.input(r_path + infile):
                positions.append(float(line))
            self.saved_r_poses[os.path.splitext(infile)[0]] = positions'''

        self.lock = threading.Lock()
        rospy.Subscriber('joint_states', JointState, self.joint_states_cb)
        
        #parsing for animations
        dir = os.path.dirname(__file__)
        qWarning(dir)
        filename = os.path.join(self.dir, 'animations/')

        ani_path = filename
        ani_listing = os.listdir(ani_path)
        for infile in ani_listing:
            pose_left = []
            pose_right = []
            for line in fileinput.input(ani_path + infile):
                pose = [float(x) for x in line.split()]
                pose_left.append(pose[:len(pose)/2])
                pose_right.append(pose[len(pose)/2:])
            self.saved_animations[os.path.splitext(infile)[0]] = Pair(pose_left, pose_right)


        # Create a trajectory action client
        r_traj_controller_name = '/r_arm_controller/joint_trajectory_action'
        self.r_traj_action_client = SimpleActionClient(r_traj_controller_name, JointTrajectoryAction)
        rospy.loginfo('Waiting for a response from the trajectory action server for RIGHT arm...')
        self.r_traj_action_client.wait_for_server()
        
        l_traj_controller_name = '/l_arm_controller/joint_trajectory_action'
        self.l_traj_action_client = SimpleActionClient(l_traj_controller_name, JointTrajectoryAction)
        rospy.loginfo('Waiting for a response from the trajectory action server for LEFT arm...')
        self.l_traj_action_client.wait_for_server()
        
        QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))
        self.joint_sig.connect(self.joint_sig_cb)
            
        # Create a large vertical box that is top aligned
        large_box = QtGui.QVBoxLayout()
        large_box.setAlignment(QtCore.Qt.AlignTop)
        large_box.setMargin(0)
        large_box.addItem(QtGui.QSpacerItem(10,0))

        # Buttons for controlling the head of the robot
        head_box = QtGui.QHBoxLayout()
        head_box.addItem(QtGui.QSpacerItem(230,0))
        head_box.addWidget(self.create_pressed_button('Head Up'))
        head_box.addStretch(1)
        large_box.addLayout(head_box)
    

        button_box = QtGui.QHBoxLayout()
        button_box.addItem(QtGui.QSpacerItem(80,0))
        button_box.addWidget(self.create_pressed_button('Head Turn Left'))
        button_box.addWidget(self.create_pressed_button('Head Down'))
        button_box.addWidget(self.create_pressed_button('Head Turn Right'))
        button_box.addStretch(1)
        button_box.setMargin(0)
        button_box.setSpacing(0)
        large_box.addLayout(button_box)
            
        # Shows what the robot says
        speech_box = QtGui.QHBoxLayout()

        self.speech_label = QtGui.QLabel('Robot has not spoken yet') #
        palette = QtGui.QPalette()
        palette.setColor(QtGui.QPalette.Foreground,QtCore.Qt.blue)
        self.speech_label.setPalette(palette) #
        speech_box.addItem(QtGui.QSpacerItem(100,0))
        #speech_box.addWidget(self.speech_label) #

        large_box.addLayout(speech_box)

        # Speak button
        speak_button_box = QtGui.QHBoxLayout();
        speech_prompt = QtGui.QLabel('Enter Speech Text:')
        speech_prompt.setFixedWidth(self.prompt_width)
        speak_button_box.addWidget(speech_prompt)
        robot_says = QtGui.QLineEdit(self._widget)
        robot_says.setFixedWidth(self.input_width)
        robot_says.textChanged[str].connect(self.onChanged) #
        speak_button_box.addWidget(robot_says)
        speak_button_box.addWidget(self.create_button('Speak'))
        speak_button_box.addStretch(1)
        large_box.addLayout(speak_button_box)

        large_box.addItem(QtGui.QSpacerItem(0,50)) 
       
        # Buttons for arm poses
        pose_button_box1 = QtGui.QHBoxLayout()
        pose_button_box1.addItem(QtGui.QSpacerItem(150,0))
        pose_button_box1.addWidget(self.create_button('Relax Left Arm'))
        pose_button_box1.addWidget(self.create_button('Relax Right Arm'))
        pose_button_box1.addStretch(1)
        large_box.addLayout(pose_button_box1)
        
        '''
        pose_button_box2 = QtGui.QHBoxLayout()
        self.l_arm_save = QtGui.QLineEdit(self._widget)
        pose_button_box2.addWidget(self.l_arm_save);
        pose_button_box2.addWidget(self.create_button('Save Left Arm Pose'))
        pose_button_box1.addItem(QtGui.QSpacerItem(275,0))
        
        self.r_arm_save = QtGui.QLineEdit(self._widget)
        pose_button_box2.addWidget(self.r_arm_save);
        pose_button_box2.addWidget(self.create_button('Save Right Arm Pose'))
        pose_button_box2.addStretch(1)
        large_box.addLayout(pose_button_box2)
        
        pose_button_box3 = QtGui.QHBoxLayout()
        self.l_arm_load = QtGui.QComboBox(self._widget)
        pose_button_box3.addWidget(self.l_arm_load)
        pose_button_box3.addWidget(self.create_button('Load Left Arm Pose'))
        self.r_arm_load = QtGui.QComboBox(self._widget)
        pose_button_box3.addWidget(self.r_arm_load)
        pose_button_box3.addWidget(self.create_button('Load Right Arm Pose'))
        pose_button_box3.addWidget(self.create_button('Load Both Arm Poses'))
        pose_button_box3.addStretch(1)
        large_box.addLayout(pose_button_box3)
        '''
        
        # Buttons for grippers
        gripper_button_box = QtGui.QHBoxLayout()
        gripper_button_box.addItem(QtGui.QSpacerItem(150,0))
        gripper_button_box.addWidget(self.create_button('Open Left Gripper'))
        gripper_button_box.addWidget(self.create_button('Open Right Gripper'))
        gripper_button_box.addStretch(1)
        large_box.addLayout(gripper_button_box)
        
        large_box.addItem(QtGui.QSpacerItem(0,25)) 
        
         # Buttons for animation
        animation_box = QtGui.QHBoxLayout()
        play_anim_label = QtGui.QLabel('Select Animation:')
        play_anim_label.setFixedWidth(self.prompt_width)
        animation_box.addWidget(play_anim_label)
        self.saved_animations_list = QtGui.QComboBox(self._widget)
        animation_box.addWidget(self.saved_animations_list)
        
        pose_time_label = QtGui.QLabel('Duration(sec):')
        pose_time_label.setFixedWidth(100)
        animation_box.addWidget(pose_time_label)
        self.pose_time = QtGui.QLineEdit(self._widget)
        self.pose_time.setFixedWidth(50)
        self.pose_time.setText('2.0')
        animation_box.addWidget(self.pose_time)
        
        animation_box.addWidget(self.create_button('Play Animation'))
        animation_box.addStretch(1)
        large_box.addLayout(animation_box)
        
        animation_box2 = QtGui.QHBoxLayout()
        animation_name_label = QtGui.QLabel('Enter Animation Name:')
        animation_name_label.setFixedWidth(self.prompt_width)
        animation_box2.addWidget(animation_name_label)
        self.animation_name = QtGui.QLineEdit(self._widget)
        self.animation_name.setFixedWidth(self.input_width)
        animation_box2.addWidget(self.animation_name)
        animation_box2.addWidget(self.create_button('Save Animation'))
        animation_box2.addStretch(1)
        large_box.addLayout(animation_box2)
        
        animation_box3 = QtGui.QHBoxLayout()
        pose_name_label = QtGui.QLabel('Enter Pose Name:')
        pose_name_label.setFixedWidth(self.prompt_width)
        animation_box3.addWidget(pose_name_label)
        self.pose_name_temp = QtGui.QLineEdit(self._widget)
        self.pose_name_temp.setFixedWidth(self.input_width)
        animation_box3.addWidget(self.pose_name_temp)
        animation_box3.addWidget(self.create_button('Add Current Pose'))
        animation_box3.addStretch(1)
        large_box.addLayout(animation_box3)
        
        # Playing around with UI stuff
        play_box = QtGui.QHBoxLayout()
        pose_sequence_label = QtGui.QLabel('Current Pose Sequence:')
        pose_sequence_label.setFixedWidth(self.prompt_width)
        pose_sequence_label.setAlignment(QtCore.Qt.AlignTop)
        
        self.list_widget = QListWidget()
        self.list_widget.setDragDropMode(QAbstractItemView.InternalMove)
        self.list_widget.setMaximumSize(self.input_width, 200)
        play_box.addWidget(pose_sequence_label)
        play_box.addWidget(self.list_widget)
        
        play_box.addStretch(1)
        large_box.addLayout(play_box)
        
        large_box.addItem(QtGui.QSpacerItem(0,50)) 
        
        # Buttons for first row of base controls
        first_base_button_box = QtGui.QHBoxLayout()
        first_base_button_box.addItem(QtGui.QSpacerItem(70,0))
        first_base_button_box.addWidget(self.create_pressed_button('Rotate Left'))
        first_base_button_box.addWidget(self.create_pressed_button('^'))
        first_base_button_box.addWidget(self.create_pressed_button('Rotate Right'))
        first_base_button_box.addStretch(1)
        large_box.addLayout(first_base_button_box)

        # Buttons for second row of base controls
        second_base_button_box = QtGui.QHBoxLayout()
        second_base_button_box.addItem(QtGui.QSpacerItem(70,0))
        second_base_button_box.addWidget(self.create_pressed_button('<'))
        #second_base_button_box.addItem(QtGui.QSpacerItem(10,0))        
        second_base_button_box.addWidget(self.create_pressed_button('v'))
        #second_base_button_box.addItem(QtGui.QSpacerItem(10,0)) 
        second_base_button_box.addWidget(self.create_pressed_button('>'))
        second_base_button_box.addStretch(1)
        large_box.addLayout(second_base_button_box)
        
        # Animation related items to store intermediate pose co-ordinates to save
        self.animation_map = {}
        
        self.create_state = False
        self.animPlay = AnimationPlayer(None, None)
        
        self._widget.setObjectName('SimpleGUI')
        self._widget.setLayout(large_box)
        context.add_widget(self._widget)

        # Look straight when launched
        self.head_x = 1.0
        self.head_y = 0.0
        self.head_z = 1.3
        self.head_action(self.head_x, self.head_y, self.head_z)

        # Set grippers to closed on initialization
        self.gripper_action('l', 0.0)
        self.gripper_action('r', 0.0)
    
        # Lab 6
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker)
        
        # Saved states for poses
        saved_pose_box = QtGui.QHBoxLayout()
        self.saved_left_poses = QtGui.QLabel('')
        self.saved_right_poses = QtGui.QLabel('')
        
        saved_pose_box.addWidget(self.saved_left_poses)
        saved_pose_box.addWidget(self.saved_right_poses)
        large_box.addLayout(saved_pose_box)
        '''
        # Preload the map of poses
        self.l_arm_load.addItems(self.saved_l_poses.keys())
        self.r_arm_load.addItems(self.saved_r_poses.keys())
        '''
        # Preload the map of animations
        self.saved_animations_list.addItems(self.saved_animations.keys())
        
        
        
    # Event for when text box is changed
    def onChanged(self, text):    
        self.speech_label.setText(text)
        self.speech_label.adjustSize()
        
    def sound_cb(self, sound_request):
        qWarning('Received sound.')
        self.sound_sig.emit(sound_request)
        
    def create_button(self, name):
        btn = QtGui.QPushButton(name, self._widget)
        btn.setFixedWidth(150)
        btn.clicked.connect(self.command_cb)
        return btn
    
    def create_pressed_button(self, name):
        btn = QtGui.QPushButton(name, self._widget)
        btn.setFixedWidth(150)
        btn.pressed.connect(self.command_cb)
        btn.setAutoRepeat(True) # To make sure the movement repeats itself
        return btn

    def sound_sig_cb(self, sound_request):
        qWarning('Received sound signal.')
        qWarning('Robot said: ' + sound_request.arg)
        self.speech_label.setText(sound_request.arg) #'Robot said: ' + 

	#a button was clicked
    def command_cb(self):
        button_name = self._widget.sender().text()
        
		#robot talk button clicked
        if (button_name == 'Speak'):
            qWarning('Robot will say: ' + self.speech_label.text() )
            self._sound_client.say(self.speech_label.text())
            self.show_text_in_rviz("Robot is Speaking")
		    
        #gripper button selected
        elif ('Gripper' in button_name):
            self.gripper_click(button_name)
        
        # Move forward
        elif (button_name == '^'):
            self.base_action(0.25, 0.0, 0.0, 0.0, 0.0, 0.0)

        # Move left
        elif (button_name == '<'):
            self.base_action(0.0, 0.25, 0.0, 0.0, 0.0, 0.0)     
        
        # Move right    
        elif (button_name == '>'):
            self.base_action(0.0, -0.25, 0.0, 0.0, 0.0, 0.0)
        
        # Move back
        elif (button_name == 'v'):
            self.base_action(-0.25, 0.0, 0.0, 0.0, 0.0, 0.0)

        #Rotate Left
        elif (button_name == 'Rotate Left'):
            self.base_action(0.0, 0.0, 0.0, 0.0, 0.0, 0.50)

        # Rotate Right
        elif (button_name == 'Rotate Right'):
            self.base_action(0.0, 0.0, 0.0, 0.0, 0.0, -0.50)   

        # A head button selected
        elif ('Head' in button_name):
            self.rotate_head(button_name)
        
        #An arm button selected
        #third param unused in freeze/relax
        #Second word in button should be side
        elif ('Arm' in button_name):

            arm_side = button_name.split()[1]
            
            if ('Freeze' in button_name or 'Relax' in button_name):
                new_arm_state = button_name.split()[0]
                self.toggle_arm(arm_side[0].lower(), new_arm_state, True)
                
                old_arm_state = ''
                if (new_arm_state == 'Relax'):
                    old_arm_state = 'Freeze'
                else:
                    old_arm_state = 'Relax'
                
                self._widget.sender().setText('%s %s Arm' % (old_arm_state, arm_side))
            '''elif ('Pose' in button_name):
                if ('Save' in button_name):
                    self.save_pose(arm_side[0].lower())
                elif ('Load' in button_name):
                    self.move_arm(arm_side[0].lower(), self.saved_l_poses[self.l_arm_load.currentText()], self.saved_r_poses[self.r_arm_load.currentText()])'''
        
        elif('Play Animation' == button_name):
            self.animPlay.left_poses = self.saved_animations[self.saved_animations_list.currentText()].left
            self.animPlay.right_poses = self.saved_animations[self.saved_animations_list.currentText()].right
            if self.pose_time.text() == '':
                self.show_warning('Please enter a duration in seconds.')
            else:
                self.animPlay.play(self.pose_time.text())
        
        elif('Animation' in button_name):
            if ('Save' in button_name):
                if self.animation_name.text() == '':
                    self.show_warning('Please enter name for animation')
                else:
                    self.save_animation(self.animation_name.text())
                    self.list_widget.clear()
                    self.animation_name.setText('')
                
        elif('Add Current Pose' == button_name):
            if self.pose_name_temp.text() == '':
                self.show_warning('Insert name for pose')
            else:
                self.animation_map[self.pose_name_temp.text()] = Pair(self.get_joint_state('l'), self.get_joint_state('r'))
                list_item = QListWidgetItem()
                list_item.setText(self.pose_name_temp.text())
                self.list_widget.addItem(list_item) 
                self.pose_name_temp.setText('')  
                    
    # gripper_type is either 'l' for left or 'r' for right
    # gripper position is the position as a parameter to the gripper goal
    def gripper_action(self, gripper_type, gripper_position):
        name_space = '/' + gripper_type + '_gripper_controller/gripper_action'
        
        gripper_client = SimpleActionClient(name_space, GripperCommandAction)
        gripper_client.wait_for_server()
        gripper_goal = GripperCommandGoal()
        gripper_goal.command.position = gripper_position 
        gripper_goal.command.max_effort = 30.0
        gripper_client.send_goal(gripper_goal)
        
    def base_action(self, x, y, z, theta_x, theta_y, theta_z):
        topic_name = '/base_controller/command'
        base_publisher = rospy.Publisher(topic_name, Twist)

        twist_msg = Twist()
        twist_msg.linear = Vector3(x, y, z)
        twist_msg.angular = Vector3(theta_x, theta_y, theta_z)
        
        base_publisher.publish(twist_msg)

    def head_action(self, x, y, z):
        name_space = '/head_traj_controller/point_head_action'
        head_client = SimpleActionClient(name_space, PointHeadAction)
        head_goal = PointHeadGoal()
        head_goal.target.header.frame_id = 'base_link'
        head_goal.min_duration = rospy.Duration(1.0)
        head_goal.target.point = Point(x, y, z)
        head_client.send_goal(head_goal)    
            
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def show_text_in_rviz(self, text):
        marker = Marker(type=Marker.TEXT_VIEW_FACING, id=0,
                lifetime=rospy.Duration(1.5),
                pose=Pose(Point(0.5, 0.5, 1.45), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.06, 0.06, 0.06),
                header=Header(frame_id='base_link'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8), text=text)
        self.marker_publisher.publish(marker)

    def show_arrow_in_rviz(self, arrow):
        marker = Marker(type=Marker.ARROW, id=0,
                lifetime=rospy.Duration(1.5),
                pose=Pose(Point(0.5, 0.5, 1.45), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.06, 0.06, 0.06),
                header=Header(frame_id='base_link'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8), arrow=arrow)
        self.marker_publisher.publish(marker)
        
    def save_animation(self, animation_name):
        if animation_name != '':
            filename = os.path.join(self.dir, 'animations/')
            anim_file = open(filename + animation_name + '.txt', 'w')
            l_animation_queue = []
            r_animation_queue = []
            for i in range(self.list_widget.count()):
                item = self.list_widget.item(i)
                pose_name = item.text()
                anim_file.write(re.sub(',' , '', str(self.animation_map[pose_name].left).strip('[]') +
                                             ' ' + str(self.animation_map[pose_name].right).strip('[]')))
                l_animation_queue.append(self.animation_map[pose_name].left)
                r_animation_queue.append(self.animation_map[pose_name].right)
                anim_file.write('\n')
            anim_file.close()
            
            self.saved_animations[animation_name] = Pair(l_animation_queue, r_animation_queue)
            self.saved_animations_list.addItem(animation_name) # Bug? Multiple entries
   
            # Reset the pending queue
            self.l_animation_queue = []
            self.r_animation_queue = []
        else:
            self.show_warning('Please insert name for animation')
        
        
    '''def save_pose(self, side_prefix):
        if (side_prefix == 'r'):
            pose_name = self.r_arm_save.text()
            if(pose_name == ''):
                self.show_warning('Please insert name for pose!')
            else:
                positions = self.get_joint_state('r')
                dir = os.path.dirname(__file__)
                filename = os.path.join(dir, '/catkin_ws/lib/rqt_simplegui/r_arm_poses/')
                qWarning(filename)
                pose_file = open(filename + pose_name + '.txt', 'w')
                for position in positions:
                    pose_file.write(str(position) + '\n')
                pose_file.close()
                self.saved_r_poses[pose_name] = positions
                self.r_arm_load.addItem(pose_name)
          
        else:
            pose_name = self.l_arm_save.text()
            if(pose_name == ''):
               self.show_warning('')
            else:
                positions = self.get_joint_state('l')
                pose_file = open('/home/team4/catkin_ws/lib/rqt_simplegui/l_arm_poses/' + pose_name + '.txt', 'w')
                for position in positions:
                    pose_file.write(str(position) + '\n')
                pose_file.close()
                self.saved_l_poses[pose_name] = positions
                self.l_arm_load.addItem(pose_name)'''

    def move_arm(self, side_prefix, left_pose, right_pose):
        if (side_prefix == 'r'):
            if right_pose == None:
                rospy.logerr('Target pose does not exist.')
            else:
                self.toggle_arm('r', 'Freeze', False)
                self.move_to_joints('r', right_pose, 2.0)
        elif (side_prefix == 'l'):
            if left_pose == None:
                rospy.logerr('Target pose does not exist.')
            else:
                self.toggle_arm('l', 'Freeze', False)
                self.move_to_joints('l', left_pose, 2.0)
                
        # Move both arms according to the selected boxes on the GUI
        else:
            if right_pose != None:
                self.toggle_arm('r', 'Freeze', False)
                self.move_to_joints('r', right_pose, 2.0)
            if left_pose != None:
                self.toggle_arm('l', 'Freeze', False)
                self.move_to_joints('l', left_pose, 2.0)
                
                
    def gripper_click(self, button_name):
        grip_side = ''
        grip_side_text = ''
        
        if ('Left' in button_name):
            grip_side = 'l'
            grip_side_text = 'left'
        else:
            grip_side = 'r'
            grip_side_text = 'right'
            
        if ('Open' in button_name):
            grip_action = 20.0
            grip_action_text = 'close'
            qWarning('Robot opened %s gripper' % (grip_side_text))
        else:
            grip_action = 0.0
            grip_action_text = 'open'
            qWarning('Robot closed %s gripper' % (grip_side_text))
            
        
        self.show_text_in_rviz("%sing %s Gripper" % (grip_action_text.capitalize(), grip_side_text.capitalize()))
        self.gripper_action(grip_side, grip_action)
        
        self._widget.sender().setText('%s %s Gripper' % (grip_action_text.capitalize(), grip_side_text.capitalize()))
            
    def rotate_head(self, button_name):
        if('Left' in button_name):
            #qWarning('x: %s, y: %s' % (self.head_x, self.head_y))
           
            if (self.head_x < -0.8 and self.head_y > 0.0):
                self.show_warning('Can\'t rotate anymore')

            elif (self.head_y < 0.0):
                self.head_x += 0.1
                self.head_y = -((1.0 - self.head_x ** 2.0) ** 0.5)
                self.show_text_in_rviz("Turning Head Left")

            else:
                self.head_x -= 0.1
                self.head_y = (1.0 - self.head_x ** 2.0) ** 0.5
                self.show_text_in_rviz("Turning Head Left")

            qWarning('x: %s, y: %s' % (self.head_x, self.head_y))
            self.head_action(self.head_x, self.head_y, self.head_z)
            
        elif('Up' in button_name):
            if (self.head_z <= 1.6):
                self.head_z += 0.1
                self.show_text_in_rviz("Moving Head Up")
                self.head_action(self.head_x, self.head_y, self.head_z)
            else:
                self.show_warning('Can\'t look up anymore')
        
        elif('Down' in button_name):
            if (self.head_z >= -2.2):
                self.head_z -= 0.1
                self.show_text_in_rviz("Moving Head Down")
                self.head_action(self.head_x, self.head_y, self.head_z)
            else:
                self.show_warning('Can\'t look down anymore') 
            
        else:
            #qWarning('x: %s, y: %s' % (self.head_x, self.head_y))
            if (self.head_x < -0.8 and self.head_y < 0.0):
                self.show_warning('Can\'t rotate anymore')

            elif (self.head_y > 0.0):
                self.head_x += 0.1
                self.head_y = (1.0 - self.head_x ** 2.0) ** 0.5
                self.show_text_in_rviz("Turning Head Right")
                
            else:
                self.head_x -= 0.1
                self.head_y = -((1.0 - self.head_x ** 2.0) ** 0.5)
                self.show_text_in_rviz("Turning Head Right")     
            
            #qWarning('x: %s, y: %s' % (self.head_x, self.head_y))
            self.head_action(self.head_x, self.head_y, self.head_z)


    def move_to_joints(self, side_prefix, positions, time_to_joint):
        '''Moves the arm to the desired joints'''
        velocities = [0] * len(positions)
        traj_goal = JointTrajectoryGoal()
        traj_goal.trajectory.header.stamp = (rospy.Time.now() + rospy.Duration(1.0))
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

    def joint_states_cb(self, msg):
        '''Callback function that saves the joint positions when a
            joint_states message is received'''
        self.lock.acquire()
        self.all_joint_names = msg.name
        self.all_joint_poses = msg.position
        self.joint_sig.emit(msg)
        self.lock.release()

    def joint_sig_cb(self, msg):
        pass

    def get_joint_state(self, side_prefix):
        '''Returns position for arm joints on the requested side (r/l)'''
        if side_prefix == 'r':
            joint_names = self.r_joint_names
        else:
            joint_names = self.l_joint_names

        if self.all_joint_names == []:
            rospy.logerr("No robot_state messages received yet!\n")
            return None
    
        positions = []
        self.lock.acquire()
        for joint_name in joint_names:
            if joint_name in self.all_joint_names:
                index = self.all_joint_names.index(joint_name)
                position = self.all_joint_poses[index]
                positions.append(position)
            else:
                rospy.logerr("Joint %s not found!", joint_name)
                self.lock.release()
                return None

        self.lock.release()
        return positions
    
    def show_warning(self, text):
        qWarning(text)
        msgBox = QMessageBox()
        msgBox.setText(text)
        msgBox.exec_()
        


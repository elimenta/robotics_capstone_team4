#Overview
PR2 project using ROS involving the collection of trash on the floor and placement into trash bins.
The contents of this repository should be in <your catkin_ws>/src, you should not have a folder called robotics_capstone_team4 (replace this with your catkin_ws/src directory)
##Installation 

1. Clone in the desktop and the robot. (git clone https://github.com/elimenta/robotics_capstone_team4.git)
   * Read the overview section about what the file hierarchy should look like.
   * Run ssh -X c1 to remote connect to the robot (required to make rqt_gui work on the robot). 
   * Perform catkin_make in your catkin_ws directory.
   * Launch the pr2_trash_collection_robot.launch on the robot located in the capstone_project/launch directory. This initializes the GUI, and navigation components on the robot.
3. On the desktop, run realrobot. Then, launch the pr2_trash_collection_desktop.launch (This launches RVIZ for map navigation and pick and place controls)
4. Once rqt has appeared, and the robot appears on RVIZ, start up SimpleGUI. 
   * After a while it will ask you to perform a 2D Pose Estimate. Do the 2D Pose Estimate on the RVIZ window containing the map, such that it matches the location of the real robot.
   
You have now completely initialized the components (assuming everything worked). Otherwise retry the above steps.
   
#Milestone 2: Running the Trash Collector
Our trash collector currently activates through a button in our GUI labeled with the words "Autonomous Demo"
This will cause the PR2 robot to navigate to fixed pre-defined positions in a map (which are its patrol points), search for trash on the ground and attempt to pick it up, then move the object to the bin.

There are two other buttons which were meant for testing purposes: "Object Detect" and "Move to Bin". Object Detect will attempt to segment an object in front of the PR2 and then continue to place that object to a predefined bin location. "Move to Bin" will cause the PR2 to move to the bin location and throw out trash.
For other test tools, read on to assignment 4

##Assignment 3: Moving to a Specified Trash Location
Our GUI has a button for moving towards a specified trash can and placing the trash into that location using the robot's left gripper. For this to work properly, we recommend playing the "left_tuck" animation before moving towards the trash bin. The object should already be on the PR2's left gripper to work. After the robot is in the "left_tuck" position, simply click the button.

##Assignment 4: Detecting Objects on Floor
The GUI also contains a button called "Detect Object" which detects objects on the floor based on what the PR2 Kinect Camera sees. This button prints out the number of clusters detected on the floor on the terminal.

Currently, there is some bad data being detected (sometimes pieces of the floor is detected as an object).
To test whether this works properly, you can use the RVIZ object manipulation menus which contain options for segmenting and picking up segmented objects. You may need to add displays for PointCloud2 and choose the topic for the kinect camera. Click the Segment button in the menu, and any clusters it sees will appear in green. You can pick up the cluster by right clicking on a green cluster and click pick up with the desired arm.

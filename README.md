#Overview
PR2 project using ROS involving the collection of trash on the floor and placement into trash bins.
The contents of this repository should be under <your catkin_ws>/src, you should not have a folder called robotics_capstone_team4 (replace this with your catkin_ws/src)
##Running the Trash Collector
Steps to run the robot:

Working with the GUI:
1. Ensure that the launch file in tabletop_segmentation.launch has been modified with the values provided to work with detection on floors.
   a. The simple way to do this is to roscd into tabletop_object_detector and change the values appropriately with the values contained in the text file.
2. Make sure the project has been cloned in the desktop and the robot. (git clone https://github.com/elimenta/robotics_capstone_team4.git)
   a. Run ssh -X c1 to remote connect to the robot (required to make rqt_gui work on the robot). 
   b. Launch the pr2_trash_collection_robot.launch on the robot located in the capstone_project/launch directory. This initializes the GUI, and navigation components on the robot.
3. On the desktop, run realrobot. Then, launch the pr2_trash_collection_desktop.launch (This launches RVIZ for map navigation and pick and place controls)
4. Once rqt has appeared, and the robot appears on RVIZ, start up SimpleGUI. 
   a. After a while it will ask you to perform a 2D Pose Estimate. Do the 2D Pose Estimate on the RVIZ window containing the map, such that it matches the location of the real robot.
   You have now completely initialized the components (assuming everything worked). Otherwise retry the above steps.
   
##Assignment 3: Moving to a Specified Trash Location
Our GUI has a button for moving towards a specified trash can and placing the trash into that location using the robot's left gripper. For this to work properly, we recommend playing the "l_tuck2" animation before moving towards the trash bin. The object should already be on the PR2's hand to work.

##Assignment 4: Detecting Objects on Floor
The GUI also contains a button called "Detect Object" which detects objects on the floor based on what the PR2 Kinect Camera sees. This button simple prints out the number of clusters detected on the floor on the terminal.

Currently, there is some bad data being detected (sometimes pieces of the floor is detected as an object).
To test whether this works properly, you can use the RVIZ object manipulation menus which contain options for segmenting and picking up segmented objects. You may need to add displays for PointCloud2 and choose the topic for the kinect camera. Click the Segment button in the menu, and any clusters it sees will appear in green. TO pick it up currently, you must right click on the green cluster and click pick up with the desired arm.
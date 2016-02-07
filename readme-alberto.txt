BEFORE RUNNING ON REAL BAXTER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
If the machine you are using to control Baxter has never connected to Baxter before (i.e. the time.bat script doesn't return a date), 

$ ssh ruser@drogon.cs.rutgers.edu

Then it will work.

========
robotiq hand
=========

rosrun robotiq_s_model_control SModelTcpNode.py 192.168.1.11

=======
Force torque sensor
========

rosrun robotiq_force_torque_sensor rq_sensor
rosrun robotiq_force_torque_sensor rq_test_sensor


================================================================================================================
PRACSYS with Motoman - APC command sequence
================================================================================================================
  
- To run Motoman as it was run during the apc
  Open 11 terminals (consider using terminator) and run in them the following commands, one in each terminal in the following sequence. Wait until each program properly start before going forward.
  $ roscore
  $ roslaunch motoman_driver robot_interface_streaming_fs100_apc.launch
  $ roslaunch prx_mapping prx_mapping_freenect_no_rviz_motoman_2_kinects.launch
  $ rosrun prx_localizer prx_localizer2
  $ roslaunch prx_planning experiment_with_sensing.launch
  In a single terminal:
  $ rosrun dynamic_reconfigure dynparam set /camera/driver depth_registration True
  $ rosrun dynamic_reconfigure dynparam set /camera2/driver depth_registration True
  $ rosrun dynamic_reconfigure dynparam set /prx_mapping_node first_rotating_axis 1
  Again, one per terminal:
  $ cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; rosrun object_recognition_core detection -c  `rospack find object_recognition_linemod`/conf/detection.ros.ork
  $ cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; rosrun object_recognition_core detection __name:=detection2 -c  `rospack find object_recognition_linemod`/conf/detection.ros_kinect2.ork
  $ rosrun prx_decision_making ru_decision_making apc.json
  $ roslaunch prx_planning motoman_planning.launch
  $ rosrun prx_planning prx_task_planning.py


- To calibrate the left arm, change the last two lines to:
  $ rosrun prx_decision_making ru_decision_making apc_1.json
  $ rosrun prx_planning calib.py  
  To change the bin, change the .json file to guide the decision making module to the proper <arm, bin> pair
  

- To gather data for object detection evaluation run all modules above changing the mapping module invocation as below:
  $ roslaunch prx_mapping prx_mapping_freenect_no_rviz_motoman_data_gathering.launch



================================================================================================================
PRACSYS with Motoman - Simulation with kinect, one arm
================================================================================================================
  
- To run Motoman PRACSYS in simulation with kinect
  Open 9 terminals (consider using terminator) and run in them the following commands, one in each terminal in the following sequence. Wait until each program properly start before going forward.
  $ roscore
  $ roslaunch prx_simulation motoman_sim.launch
  $ roslaunch prx_mapping prx_mapping_freenect_no_rviz_motoman.launch
  $ rosrun prx_localizer prx_localizer2
  In a single terminal:
  $ rosrun dynamic_reconfigure dynparam set /camera/driver depth_registration True
  $ rosrun dynamic_reconfigure dynparam set /prx_mapping_node first_rotating_axis 1
  Again, one per terminal:
  $ cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; rosrun object_recognition_core detection -c  `rospack find object_recognition_linemod`/conf/detection.ros.ork
  $ rosrun prx_decision_making ru_decision_making apc_2.json
  $ roslaunch prx_planning motoman_planning.launch
  $ rosrun prx_planning prx_task_planning.py



================================================================================================================
Bare bones simulation (no sensing) - MoveIt!
================================================================================================================
  
- To run simulations without sensing
  Open 4 terminals and run in them the following commands, one in each terminal in the following sequence. Wait until each program properly start before going forward.
  $ roscore
  $ roslaunch prx_simulation motoman_sim.launch
  $ roslaunch prx_mapping prx_mapping_no_mapping_motoman.launch
  $ rosrun prx_decision_making prx_decision_making_node example2.json
  $ roslaunch motoman_sda10f_moveit_config moveit_apc_motoman_sim.launch
  $ cd $PRACSYS_PATH/../object_models; rosrun prx_motion_planning3 prx_motion_planning3.py



================================================================================================================
MoveIt! with Motoman - 9 terminals
================================================================================================================
  
- To control the Motoman simulator with MoveIt - One arm
  Open 9 terminals and run in them the following commands, one in each terminal (unless recomended otherwise) in the following sequence. 
  Wait until each program properly start before going forward.
  $ roscore
  $ roslaunch motoman_driver robot_interface_streaming_fs100_apc.launch
  $ roslaunch prx_mapping prx_mapping_freenect_no_rviz_motoman.launch
  $ rosrun prx_localizer prx_localizer
  In a single terminal:
  $ rosrun dynamic_reconfigure dynparam set /camera/driver data_skip 10
  $ rosrun dynamic_reconfigure dynparam set /prx_mapping_node first_rotating_axis 1
  $ rosrun dynamic_reconfigure dynparam set /camera/driver depth_registration True
  Again, one per terminal:
  $ cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; rosrun object_recognition_core detection -c  `rospack find object_recognition_linemod`/conf/detection.ros.ork
  $ rosrun prx_decision_making prx_decision_making_node example2.json
  $ roslaunch motoman_sda10f_moveit_config moveit_apc_motoman_sim.launch
  $ cd $PRACSYS_PATH/../object_models; rosrun prx_motion_planning3 prx_motion_planning3.py
 


================================================================================================================
MoveIt! with Motoman Simulator - 4 terminals
================================================================================================================
  
- To control the Motoman simulator with MoveIt - One arm
  Open 4 terminals and run the following commands in them, one in each terminal in the following sequence. Wait until each program properly start before going forward.
  $ roscore
  $ roslaunch prx_simulation motoman_sim.launch
  $ cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; roslaunch prx_mapping all_but_motion_planning_motoman.launch
  $ cd $PRACSYS_PATH/../object_models; rosrun prx_motion_planning3 prx_motion_planning3.py



================================================================================================================
MoveIt! with Motoman - 4 terminals
================================================================================================================
  
- To control the Motoman simulator with MoveIt - One arm
  Open 4 terminals and run the following commands in them, one in each terminal in the following sequence. Wait until each program properly start before going forward.
  $ roscore
  $ roslaunch motoman_driver robot_interface_streaming_fs100_apc.launch
  $ cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; roslaunch prx_mapping all_but_motion_planning_motoman.launch
  $ cd $PRACSYS_PATH/../object_models; rosrun prx_motion_planning3 prx_motion_planning3.py



================================================================================================================
MoveIt! with Baxter Simulator - 4 terminals
================================================================================================================
  
- To control the Baxter simulator with MoveIt - Two arms
  Open 4 terminals and run in them the following commands, one in each terminal in the following sequence. Wait until each program properly start before going forward.
  $ roscore
  $ roslaunch baxter_gazebo baxter_world.launch
  $ cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; roslaunch prx_mapping all_but_motion_planning_2kinects.launch
  $ cd $PRACSYS_PATH/../object_models; rosrun prx_motion_planning3 prx_motion_planning3.py


- To control the Baxter simulator with MoveIt - One arm
  Open 4 terminals and run the following commands in them, one in each terminal in the following sequence. Wait until each program properly start before going forward.
  $ roscore
  $ roslaunch baxter_gazebo baxter_world.launch
  $ cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; roslaunch prx_mapping all_but_motion_planning.launch
  $ cd $PRACSYS_PATH/../object_models; rosrun prx_motion_planning3 prx_motion_planning3.py



================================================================================================================
MoveIt! with Baxter Simulator - 12 and 11 terminals
================================================================================================================

- To control the Baxter simulator with MoveIt - Two arms
  Open 12 terminals and run in them the following commands, one in each terminal (unless recomended otherwise) in the following sequence. 
  Wait until each program properly start before going forward.
  $ roscore
  $ roslaunch baxter_gazebo baxter_world.launch
  In a single terminal:
  $ rosrun dynamic_reconfigure dynparam load /robot/left_joint_position_controller/joints/left_s1_controller/pid robot_s1.yaml
  $ rosrun dynamic_reconfigure dynparam load /robot/left_joint_position_controller/joints/left_s0_controller/pid robot_s0.yaml
  $ rosrun dynamic_reconfigure dynparam load /robot/right_joint_position_controller/joints/right_s1_controller/pid robot_s1.yaml
  $ rosrun dynamic_reconfigure dynparam load /robot/right_joint_position_controller/joints/right_s0_controller/pid robot_s0.yaml
  Again, one per terminal:
  $ roslaunch prx_mapping prx_mapping_freenect_no_rviz_2_kinects.launch
  $ rosrun prx_localizer prx_localizer
  Again in a single terminal:
  $ rosrun dynamic_reconfigure dynparam set /camera/driver depth_registration True
  $ rosrun dynamic_reconfigure dynparam set /camera/driver data_skip 10
  $ rosrun dynamic_reconfigure dynparam set /camera2/driver depth_registration True
  $ rosrun dynamic_reconfigure dynparam set /camera2/driver data_skip 10
  $ rosrun dynamic_reconfigure dynparam set /prx_mapping_node first_rotating_axis 1
  Again, one per terminal:
  $ cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; rosrun object_recognition_core detection -c  `rospack find object_recognition_linemod`/conf/detection.ros.ork
  $ cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; rosrun object_recognition_core detection __name:=detection2 -c  `rospack find object_recognition_linemod`/conf/detection.ros_kinect2.ork
  $ rosrun prx_decision_making prx_decision_making_node example2_kinect2.json
  $ rosrun baxter_interface joint_trajectory_action_server.py
  $ roslaunch baxter_moveit_config moveit_apc_baxter.launch
  $ cd $PRACSYS_PATH/../object_models; rosrun prx_motion_planning3 prx_motion_planning3.py


- To control the Baxter simulator with MoveIt - One arm
  Open 11 terminals and run in them the following commands, one in each terminal (unless recomended otherwise) in the following sequence. 
  Wait until each program properly start before going forward.
  $ roscore
  $ roslaunch baxter_gazebo baxter_world.launch
  In a single terminal:
  $ rosrun dynamic_reconfigure dynparam load /robot/left_joint_position_controller/joints/left_s1_controller/pid robot_s1.yaml
  $ rosrun dynamic_reconfigure dynparam load /robot/left_joint_position_controller/joints/left_s0_controller/pid robot_s0.yaml
  Again, one per terminal:
  $ roslaunch prx_mapping prx_mapping_freenect_no_rviz.launch
  $ rosrun prx_localizer prx_localizer
  In a single terminal:
  $ rosrun dynamic_reconfigure dynparam set /camera/driver depth_registration True
  $ rosrun dynamic_reconfigure dynparam set /prx_mapping_node first_rotating_axis 1
  $ rosrun dynamic_reconfigure dynparam set /camera/driver data_skip 10
  Again in a single terminal:
  $ cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; rosrun object_recognition_core detection -c  `rospack find object_recognition_linemod`/conf/detection.ros.ork
  $ rosrun prx_decision_making prx_decision_making_node example2.json
  $ rosrun baxter_interface joint_trajectory_action_server.py
  $ roslaunch baxter_moveit_config moveit_apc_baxter.launch
  $ cd $PRACSYS_PATH/../object_models; rosrun prx_motion_planning3 prx_motion_planning3.py
 


================================================================================================================
Baxter with MoveIt! - 4 terminals
================================================================================================================

- To control the Baxter with MoveIt - Two arms
  Open 4 terminals and run in all of them:
  $ ./drogon.sh    {or ./drogon.sh drogon.cs.rutgers.edu 172.16.71.21 if not in tormund. Use the yp of the machine you are using}
  
- Then, run the following commands, one in each terminal in the following sequence. Wait until each program properly start before going forward.
  $ ./prepare.sh {or rosrun baxter_tools enable_robot.py -e}
  $ ./time.bat
  $ cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; roslaunch prx_mapping all_but_motion_planning_baxter_2kinects.launch
  $ cd $PRACSYS_PATH/../object_models; rosrun prx_motion_planning3 prx_motion_planning3.py


- To control the Baxter with MoveIt - One arm
  Open 4 terminals and run in all of them:
  $ ./drogon.sh    {or ./drogon.sh drogon.cs.rutgers.edu 172.16.71.21 if not in tormund. Use the yp of the machine you are using}
  
- Then, run the following commands, one in each terminal in the following sequence. Wait until each program properly start before going forward.
  $ ./prepare.sh {or rosrun baxter_tools enable_robot.py -e}
  $ ./time.bat
  $ cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; roslaunch prx_mapping all_but_motion_planning_baxter.launch
  $ cd $PRACSYS_PATH/../object_models; rosrun prx_motion_planning3 prx_motion_planning3.py



================================================================================================================
MoveIt! with Motoman Simulator - 12 and 9 terminals
================================================================================================================

- To control the Baxter simulator with MoveIt - Two arms
  Open 12 terminals and run in them the following commands, one in each terminal (unless recomended otherwise) in the following sequence. 
  Wait until each program properly start before going forward.
  $ roscore
  $ roslaunch prx_simulation motoman_sim.launch
  $ roslaunch prx_mapping prx_mapping_freenect_no_rviz_motoman_2_kinects.launch
  $ rosrun prx_localizer prx_localizer
  Again in a single terminal:
  $ rosrun dynamic_reconfigure dynparam set /camera/driver depth_registration True
  $ rosrun dynamic_reconfigure dynparam set /camera/driver data_skip 10
  $ rosrun dynamic_reconfigure dynparam set /camera2/driver depth_registration True
  $ rosrun dynamic_reconfigure dynparam set /camera2/driver data_skip 10
  $ rosrun dynamic_reconfigure dynparam set /prx_mapping_node first_rotating_axis 1
  Again, one per terminal:
  $ cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; rosrun object_recognition_core detection -c  `rospack find object_recognition_linemod`/conf/detection.ros.ork
  $ cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; rosrun object_recognition_core detection __name:=detection2 -c  `rospack find object_recognition_linemod`/conf/detection.ros_kinect2.ork
  $ rosrun prx_decision_making prx_decision_making_node example2_kinect2.json
  $ roslaunch motoman_sda10f_moveit_config moveit_apc_motoman_sim.launch
  $ cd $PRACSYS_PATH/../object_models; rosrun prx_motion_planning3 prx_motion_planning3.py


- To control the Motoman simulator with MoveIt - One arm
  Open 9 terminals and run in them the following commands, one in each terminal (unless recomended otherwise) in the following sequence. 
  Wait until each program properly start before going forward.
  $ roscore
  $ roslaunch prx_simulation motoman_sim.launch
  $ roslaunch prx_mapping prx_mapping_freenect_no_rviz_motoman.launch
  $ rosrun prx_localizer prx_localizer
  In a single terminal:
  $ rosrun dynamic_reconfigure dynparam set /camera/driver depth_registration True
  $ rosrun dynamic_reconfigure dynparam set /prx_mapping_node first_rotating_axis 1
  $ rosrun dynamic_reconfigure dynparam set /camera/driver data_skip 10
  Again, one per terminal:
  $ cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; rosrun object_recognition_core detection -c  `rospack find object_recognition_linemod`/conf/detection.ros.ork
  $ rosrun prx_decision_making prx_decision_making_node example2.json
  $ roslaunch motoman_sda10f_moveit_config moveit_apc_motoman_sim.launch
  $ cd $PRACSYS_PATH/../object_models; rosrun prx_motion_planning3 prx_motion_planning3.py
 

roslaunch motoman_driver robot_interface_streaming_fs100_apc.launch

===========================++++++++++++++++++=======================++++++++++++++++++++++======================
Legacy stuff
===========================++++++++++++++++++=======================++++++++++++++++++++++======================

================================================================================================================
Compilation for debug and for Eclipse
================================================================================================================

- For debug
  $ source devel/setup.sh
  $ catkin_make -DCMAKE_BUILD_TYPE=Debug
  
- For Eclipse
  $ catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles"
  $ awk -f $(rospack find mk)/eclipse.awk ./build/.project > ./build/.project_with_env
  $ mv ./build/.project_with_env ./build/.project
  $ source devel/setup.sh
  After that, read http://wiki.ros.org/IDEs  Section 3 for Eclipse
  


================================================================================================================
Parameter setting
================================================================================================================

- Run the relevant modules and:
  $ rosrun rqt_reconfigure rqt_reconfigure



================================================================================================================
PRELIMINARY DECISION MAKING MODULE
================================================================================================================

- Decision making module's messages testing
  $ rostopic list
  $ rosmsg show prx_decision_making/DecisionMakingStateMessage
  $ rostopic hz /decision_making_state
  $ rostopic echo /decision_making_state
  


================================================================================================================
PRELIMINARY MOTION PLANNER
================================================================================================================

- To use the motion planner you have to:
  Turn Baxter on.
  
- To run the preliminary motion planning module 
  $ rosrun prx_motion_planning prx_motion_planning.py 



================================================================================================================
MAPPER
================================================================================================================

- To run the prx_mapping module:
  $ roscore {in a separate terminal}
  $ source source devel/setup.sh
  $ roslaunch prx_mapping prx_mapping.launch
  $ rosrun rqt_reconfigure rqt_reconfigure {in a separate terminal; and, in camera->driver, check depth_registration}
  
- To control the mapping module
  Normal
  $ rosrun dynamic_reconfigure dynparam set /prx_mapping_node first_rotating_axis 1
  Reset
  $ rosrun dynamic_reconfigure dynparam set /prx_mapping_node first_rotating_axis 2



================================================================================================================
OBJECT RECOGNIZER (old)
================================================================================================================

- To run:
  $ roslaunch prx_object_detection prx_object_detection.launch
  Or
  $ roslaunch prx_object_detection prx_object_detection_no_openni.launch
  Or
  $ roslaunch prx_mapping prx_mapping_and_object_detection.launch
  
  

================================================================================================================
Baxter with MoveIt!
================================================================================================================
  

- To control the Baxter with MoveIt 2
  Open 6 terminals and run in all of them 
  $ ./drogon.sh    {or ./drogon.sh drogon.cs.rutgers.edu 172.16.71.21 if not in tormund. Use the yp of the machine you are using}
  Then, run the following commands, one in each terminal in the following sequence. Wait until each program properly start before going forward.
  $ ./time.bat
  $ ./prepare.sh {or rosrun baxter_tools enable_robot.py -e}
  $ cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; roslaunch prx_mapping prx_mapping_and_object_detection2.launch
  $ rosrun prx_decision_making prx_decision_making_node example_baxter.json 
  $ roslaunch baxter_moveit_config moveit_apc_baxter.launch
  $ cd $PRACSYS_PATH/../object_models; rosrun prx_motion_planning3 prx_motion_planning3.py
  
  

================================================================================================================
Baxter
================================================================================================================

- To run the system in Baxter, open 9 terminals and run in all of them:
  $ ./drogon.sh
  
- Then, run the following commands, one in each terminal in the following sequence. Wait until each program properly start before going forward.
  $ ./time.bat
  $ ./prepare.sh {or rosrun baxter_tools enable_robot.py -e}
  $ rosrun rviz rviz --display-config baxter.rviz
  $ roslaunch prx_mapping prx_mapping_freenect_no_rviz.launch
  $ rosrun prx_localizer prx_localizer
  $ rosrun dynamic_reconfigure dynparam set /camera/driver depth_registration True
  This object detection module:
  $ roslaunch prx_object_detection prx_object_detection_no_openni.launch
  Or this object detection module:
  $ cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; rosrun object_recognition_core detection -c  `rospack find object_recognition_linemod`/conf/detection.ros.ork
  $ rosrun prx_decision_making prx_decision_making_node example2.json
  $ rosrun prx_motion_planning prx_motion_planning.py
  At this point, you will see the robot execute the full cycle. Wait for the first mapping for proper rviz output during the full cycle.


- To read arm position 
  $ python baxter_arms_position.py
  
  
- Baxter SDK
  http://sdk.rethinkrobotics.com/wiki/Workstation_Setup
  
- Baxter Hello World
  http://sdk.rethinkrobotics.com/wiki/Hello_Baxter
  
- Baxter record and replay
  http://sdk.rethinkrobotics.com/wiki/Joint_Trajectory_Playback_Example

 

================================================================================================================
Baxter Simulator
================================================================================================================

- Install: sdk.rethinkrobotics.com/wiki/Simulator_Installation
- Run: http://sdk.rethinkrobotics.com/wiki/Baxter_Simulator
- rviz: https://github.com/RethinkRobotics/sdk_docs_archive/wiki/rviz

- To speedup simulation, in gazebo change Physics -> real time update rate -> 2,000.0
- To smooth movement in simulation, in gazebo change Physics -> max step size -> 0.0030

- To run, open 9 terminals in the workspace and run the following command in all of them:
  $ ./baxter.sh sim
  Then, run the following commands, one in each terminal in the following sequence. Wait until each program properly start before going forward.
  $ roscore
  $ roslaunch baxter_gazebo baxter_world.launch
  $ rosrun rviz rviz --display-config baxter.rviz
  $ roslaunch prx_mapping prx_mapping_freenect_no_rviz.launch
  $ rosrun prx_localizer prx_localizer
  $ rosrun dynamic_reconfigure dynparam set /camera/driver depth_registration True
  This object detection module:
  $ roslaunch prx_object_detection prx_object_detection_no_openni.launch
  Or this object detection module:
  $ cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; rosrun object_recognition_core detection -c  `rospack find object_recognition_linemod`/conf/detection.ros.ork
  $ rosrun prx_decision_making prx_decision_making_node example2.json
  $ rosrun prx_motion_planning prx_motion_planning.py
  At this point, you will see the robot execute the full cycle. Wait for the first mapping for proper rviz output during the full cycle.


- To run with two machines
  - In the Simulator machine, open 3 terminals in the workspace and run the following command in all of them (the ips are the Simulator machine's ip):
  $ ./drogon.sh 192.168.1.10 192.168.1.10
  Then, run the following commands, one in each terminal in the following sequence. Wait until each program properly start before going forward.
  $ roscore
  $ roslaunch baxter_gazebo baxter_world.launch
  $ rosrun rviz rviz --display-config baxter.rviz

  - In the Perception machine, open 3 terminals in the workspace and run the following command in all of 
    them (the first ip is the Simulator machine's ip, while the second is the Perception machine's ip):
  $ ./drogon.sh 192.168.1.10 192.168.1.9
  Then, run the following commands, one in each terminal in the following sequence. Wait until each program properly start before going forward.
  $ roslaunch prx_mapping prx_mapping_freenect_no_rviz.launch
  $ rosrun prx_localizer prx_localizer
  $ rosrun dynamic_reconfigure dynparam set /camera/driver depth_registration True
  $ cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; rosrun object_recognition_core detection -c  `rospack find object_recognition_linemod`/conf/detection.ros.ork
  $ rosrun prx_decision_making prx_decision_making_node example2.json
  $ rosrun prx_motion_planning prx_motion_planning.py
  At this point, you will see the robot execute the full cycle. Wait for the first mapping for proper rviz output during the full cycle.
  rviz may fail in the first cycle. If so, reinitiate it.


- To speedup simulation
  - In gazebo, Physics -> max_step_size -> 0.003
  - For prx_mapping -> rosrun rqt_reconfigure rqt_reconfigure -> driver -> data_skip -> 10
    $ rosrun dynamic_reconfigure dynparam set /camera/driver data_skip 10 



================================================================================================================
MoveIt! with Baxter Simulator
================================================================================================================
  
- To control the Baxter simulator with MoveIt 2
  Open 7 terminals and run in all of them 
  $ ./baxter.sh sim
  Then, run the following commands, one in each terminal in the following sequence. Wait until each program properly start before going forward.
  $ roscore
  $ roslaunch baxter_gazebo baxter_world.launch
  $ cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; roslaunch prx_mapping prx_mapping_and_object_detection2.launch
  $ rosrun prx_decision_making prx_decision_making_node example2.json
  $ rosrun baxter_interface joint_trajectory_action_server.py
  $ roslaunch baxter_moveit_config moveit_apc_baxter.launch
  $ cd $PRACSYS_PATH/../object_models; rosrun prx_motion_planning3 prx_motion_planning3.py

  
 

- To control the Baxter simulator with MoveIt
  Open 8 terminals and run in all of them 
  $ ./baxter.sh sim
  Then, run the following commands, one in each terminal in the following sequence. Wait until each program properly start before going forward.
  $ roscore
  $ roslaunch baxter_gazebo baxter_world.launch
  $ rosrun baxter_tools enable_robot.py -e
  $ rosrun baxter_interface joint_trajectory_action_server.py
  $ roslaunch baxter_moveit_config demo_sim.launch
  $ python add_cube_scene_baxter.py
  

- To control the Baxter simulator with MoveIt Commander (moveit_commander_cmdline.py -> http://moveit.ros.org/wiki/MoveIt_Commander)
  Open 8 terminals and run in all of them 
  $ ./baxter.sh sim
  Then, run the following commands, one in each terminal in the following sequence. Wait until each program properly start before going forward.
  $ roscore
  $ roslaunch baxter_gazebo baxter_world.launch
  $ rosrun baxter_tools enable_robot.py -e
  $ rosrun baxter_interface joint_trajectory_action_server.py
  $ rosrun tf static_transform_publisher 0 0 0 0 0 0 world base 100
  $ roslaunch baxter_moveit_config moveit_apc_baxter.launch
  $ rosrun moveit_commander moveit_commander_cmdline.py
  In this last terminal, type
  > use left_arm
  Then go to http://moveit.ros.org/wiki/MoveIt_Commander to learn more (skip goal[0] = 0.2  It looks like it is unreachable).
  
- To test MoveIt! run everything above except the last command. Change it to:
  $ python move_group_python_interface_tutorial.py
  

- Baxter simulation with MoveIt!, open 12 terminals in the workspace and run the following command in all of them:
  $ ./baxter.sh sim
  Then, run the following commands, one in each terminal in the following sequence. Wait until each program properly start before going forward.
  $ roscore
  $ roslaunch baxter_gazebo baxter_world.launch
  $ roslaunch prx_mapping prx_mapping_freenect_no_rviz.launch
  $ rosrun prx_localizer prx_localizer
  $ rosrun dynamic_reconfigure dynparam set /camera/driver depth_registration True
  $ rosrun dynamic_reconfigure dynparam set /prx_mapping_node first_rotating_axis 1
  $ cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; rosrun object_recognition_core detection -c  `rospack find object_recognition_linemod`/conf/detection.ros.ork
  $ rosrun prx_decision_making prx_decision_making_node example2.json
  $ rosrun baxter_interface joint_trajectory_action_server.py
  $ roslaunch baxter_moveit_config moveit_apc_baxter.launch kinect:=true
  $ cd $PRACSYS_PATH/../object_models; rosrun prx_motion_planning3 prx_motion_planning3.py


- To add objects in the planning scene
  $ python src/moveit_commander/demos/pick.py
  $ python add_cube_scene_baxter.py


- MoveIt with the pr2 robot 
  Open 3 terminals and run in all of them 
  $ ./baxter.sh sim
  Then,
  $ roscore
  $ roslaunch pr2_moveit_config demo.launch
  $ python add_cube_scene_pr2.py

- Pick works! Run the three commands above and then:
  $ python src/moveit_commander/demos/pick_pr2.py
  
- To use a bag instead of kinect
  $ rosbag play 2015-02-17-15-26-59.bag
      
  

================================================================================================================
BLORT object detector
================================================================================================================

- To run, open 3 terminals in the workspace and run the following command in all of them:
  $ source devel/setup.sh
  The, run the following commands, one in each terminal in the following sequence. Wait until each program properly start before going forward.
  $ roscore
  $ roslaunch openni_launch openni.launch
  $ roslaunch blort_ros tracking.launch
  
 

================================================================================================================
Creating a log (rosbag)
================================================================================================================

- To create a log, run the APC system (simulated or real (see above)) and, in a separated terminal:
  $ cd workspace
  $ ./baxter.sh sim {simulated}
  $ ~/drogon.sh {real system}
  $ rosbag record tf /camera/depth_registered/image_raw /camera/depth_registered/camera_info /camera/depth_registered/points /camera/rgb/image_color /decision_making_state /motion_planner_state
  
- To run a log
  $ cd <workspace>
  $ rosrun rviz rviz --display-config baxter.rviz
  $ rosrun prx_mapping prx_mapping_node
  $ roslaunch prx_object_detection prx_object_detection_no_openni.launch
  $ rosbag play 2014-12-25-21-35-20.bag --clock -r 0.2
  
  Could be necessary (http://answers.ros.org/question/123256/sync-ros-bag-timestamps-with-ros-system/):
  $ rosparam set use_sim_time false
  
- To create a log containing only the kinect info, connect the kinect and run only
  $ roslaunch openni_launch openni.launch
  $ rosbag record /tf /camera/depth_registered/points /camera/depth_registered/image_raw /camera/depth_registered/camera_info /camera/rgb/image_color /camera/rgb/image_rect_color /camera/rgb/camera_info
  
- To filter a bag
  $ rosbag filter 2015-02-17-15-26-59.bag filtered.bag "topic == '/tf' or topic == '/camera/depth_registered/points' or topic == '/camera/depth_registered/image_raw' or topic == '/camera/depth_registered/camera_info' or topic == '/camera/rgb/image_color' or topic == '/camera/rgb/image_rect_color' or topic == '/camera/rgb/camera_info'"
  
- To run a bag continuously
  $ rosbag play -l filtered.bag
  
  
  
================================================================================================================
TODO
================================================================================================================

- Baixar o ros-drivers/freenect_stack e o ros-drivers/libfreenect e modificar para alterar o auto exposure dinamicamente
- Ver se vale a pena mudar a intensidade do projetor
- Ver se nao tem um corte de distancia minima em algum lugar

- Testar o stick notes com a iluminacao no kinect
- Estudar se o move to detect vai funcionar (o mapa vai ficar bom o suficiente?)
- Estudar o teste de desempenho
- Estudar o detector VG-RAM


- Nao esta obedecendo a restricao (constraint)...
- Por que o objecto graspado (lilas) esta ficando rodado?



- Verificar por que o motion planning nao inicia corretamente com o bracco direito

- Colocar todos os buffers de mensagem com tamanho 1

- Testar codigo MoveIt! no baxter. Verificar se o gripper abre e fecha no rviz.
  - Tem que testar o comando em ./gripper_config.bat com parametros do Baxter. 
    Examinar pagina: http://sdk.rethinkrobotics.com/wiki/Gripper_Customization 

- Entender por que o novo launch do moveit funciona e o velho engasga o rviz

- Verificar se nao eh melhor publicar a transform world->base no URDF (SRDF?)
  
- Verificar se trocando ns = "ExternalTools/left/PositionKinematicsNode/IKService" pelo usdo pelo Moveit da certo misturar IK com MoveIt
- Checar o correto uso de stamp e frame_id no mapper -> /point_cloud nao deveria estar ancorada em /world e world em /base?
  Isso muda muitos sistemas de coordenada...
  
- Comparar os arquivos abaixo recursivamente:
/opt/ros/hydro/share/pr2_moveit_config/launch/demo.launch
com
./src/moveit_robots/baxter/baxter_moveit_config/launch/move_group_apc.launch
- O arquivo ./src/moveit_robots/baxter/baxter_moveit_config/launch/move_group_apc.launch chama os tres arquivos abaixo 
  hierarquicamente. Os quatro tem que ser copiados para arquivos com outros nomes para serem alterados hierarquicamente.

- O arquivo ./src/moveit_robots/baxter/baxter_moveit_config/launch/sensor_manager_apc.launch chama os dois arquivos abaixo 
  hierarquicamente. Os tres tem que ser copiados para arquivos com outros nomes para serem alterados hierarquicamente.
  
- O arquivo ./src/moveit_robots/baxter/baxter_moveit_config/launch/baxter_moveit_sensor_manager_apc.launch
  deve ser alterado para:
  - Indicar a tf do octomap
  - Nao invocar o openni.launch
  - Nao publicar a tf da pose do kinect "camera_link" -> O sistema moveit deve usar a pose do kinect 
    do localizer ou o camera_link mesmo? Acho que o camera_link com relacao ao torso... Isso por que o mapa esta ancorado 
    em camera_link... Ver se torso ee igual a base (pose)
  - Ter outro parametro de resolucao do octomap
  
  
- Revisar as transformadas do localizer. 
  - Usar o mesmo mecanismo empregado no object detector para computar os time stamps do localizer (pensar, pois tem casos 
    em que deve ser melhor usar o menor de dois timestamps -> ver como computar).
  - Remover o subscribe ao depth se o indicado acima funcionar

- Mapper
  - Usar o mesmo mecanismo empregado no object detector para computar o time stamp do mapper (pensar para ver se ee isso mesmo
    ja que temos o time stamp do depth vindo do kinect).

- In object_detection
  Testar com o Senz3D
  
- Estudar a documentacao de Baxter
- Implementar o movimento da cabecca de Baxter (fazer ele olhar para a mao)

- In Localizer
  Criar parametros para inserir a calibracao da posicao fisica do kinect

- In Mapper (para Senz3D)
  Resolver referencia Tf que nao deixa ver a nuvem de kinfu (partir de do launch de Senz3D <node pkg="tf" type="static_transform_publisher" name="softkinect_tf" args="0 0 0 0 0 1.2 /base /softkinetic_camera_link 40" />
  Testar kinect com resolucao menor (http://answers.ros.org/question/9318/how-to-change-the-kinect-point-cloud-resolution/)
  Resolver alinhamento cor x depth (parametro em algum lugar em launch ou equivalente neste nivel. Tf?). Ver alinhamento kinect.
  Implementar get_average_neighbors() em prx_mapping.cpp
  Tratar tamanho da imagem-depth automaticamente
  Usar predicao da pose para nao se perder

- Sensor
  Calibrar o kinect
  Calibrar o Senz3D

- Baxter Simulator
  Incluir o modelo do kinect no modelo de Baxter para facilitar a calibracao
    O modelo e como incluir disponiveis aqui sao um ponto de partida http://www.pirobot.org/blog/0023/#Testing_the_TurtleBot_URDF_Model

- Useful commands
rosrun rqt_reconfigure rqt_reconfigure
roslaunch motoman_sda10f_moveit_config setup_assistant.launch
thg log src/linemod/src/linemod_train.cpp

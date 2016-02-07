

JonSnow

roscore
roslaunch motoman_driver robot_interface_streaming_fs100_apc.launch
roslaunch prx_planning experiment_with_sensing.launch
roslaunch prx_mapping prx_mapping_freenect_no_rviz_motoman_2_kinects.launch
rosrun prx_localizer prx_localizer2

rosrun dynamic_reconfigure dynparam set /camera/driver depth_registration True
rosrun dynamic_reconfigure dynparam set /camera2/driver depth_registration True
rosrun dynamic_reconfigure dynparam set /prx_mapping_node first_rotating_axis 1

cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; rosrun object_recognition_core detection -c  `rospack find object_recognition_linemod`/conf/detection.ros.ork
cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; rosrun object_recognition_core detection __name:=detection2 -c  `rospack find object_recognition_linemod`/conf/detection.ros_kinect2.ork

roslaunch prx_planning motoman_planning.launch
rosrun prx_decision_making ru_decision_making apc.json
rosrun prx_planning prx_task_planning.py






ICRA 15:

For Simulation:

roslaunch prx_simulation motoman_sim.launch
roslaunch prx_mapping prx_mapping_no_mapping_motoman.launch
rosrun prx_decision_making icra_decision_making example2.json
roslaunch prx_planning chuples_planning.launch
rosrun prx_planning grasp_eval_planning.py



For real world:

roslaunch motoman_driver robot_interface_streaming_fs100_apc.launch
roslaunch prx_mapping prx_mapping_no_mapping_motoman.launch
rosrun prx_decision_making icra_decision_making example2.json
roslaunch prx_planning chuples_planning.launch
rosparam set driver_node/network_interface eth1
roslaunch reflex reflex.launch
rosservice call /zero_fingers
rosrun prx_planning grasp_eval_planning.py

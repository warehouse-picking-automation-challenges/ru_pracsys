.. _ecto_ros:

ecto_ros: ROS Integration & Bagfile Support
===========================================

``ecto_ros`` contains publishers, subscribers, bag readers and bag
writers for many common ROS message types.  You can place subscribers
as the inputs to your graph and publishers at the outputs, using
converter cells as necessary (for instance, to get from Image messages
to ``cv::Mat`` objects).  

.. toctree::
   :maxdepth: 2

   ecto_ros
   std_msgs
   sensor_msgs
   geometry_msgs
   nav_msgs

.. index:: 
   :double: Remapping; ROS topics 

Remapping of ros topics
-----------------------

.. highlight:: py

ROS topic remapping works much like it does anywhere else.  Ecto
scripts that contain ROS nodes will contain the line::

  ecto_ros.init(sys.argv, "extract_largest_cluster")
  
which will do remapping as expect in ROSland.  So an example of
commandline remapping would be:

.. highlight:: ectosh

::

  ./my_script.py /input:=/camera/rgb/image


  
  



import rospy
import std_msgs.msg
import baxter_interface

rospy.init_node('motion_planner', anonymous=True)

limb = baxter_interface.Limb('right')
g_origin_robot_position = limb.joint_angles()
print "right arm:", g_origin_robot_position
print

limb = baxter_interface.Limb('left')
g_origin_robot_position = limb.joint_angles()
print "left arm:", g_origin_robot_position

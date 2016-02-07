import rospy
from moveit_python import *

rospy.init_node("moveit_py")
# create a planning scene interface, provide name of root link
p = PlanningSceneInterface("/base")

print "ps"

# add a cube of 0.1m size, at [1, 0, 0.5] in the base_link frame
p.addCube("my_cube", 0.5, 1.0, 0.5, 0.2, wait=False)
p.waitForSync(100)

print "Added"

# do something

co = p.getKnownCollisionObjects()
print co
#p.setColor("my_cube", 0.5, 0, 0)
#p.sendColors()
rospy.sleep(1)

# remove the cube
p.removeCollisionObject("my_cube")
#p.waitForSync(100)

co = p.getKnownCollisionObjects()
print co

print "Removed"

rospy.spin()
roscpp_shutdown()

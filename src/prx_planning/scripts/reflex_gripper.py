import rospy
from std_srvs.srv import Empty
 
# Imports the code to control the hand using smart actions
print "importing"
from reflex import reflex_smarts
# rospy.init_node('ExampleHandNode')
print "importing"

# To set the correct ethernet interface used for communication with the 
# ReFlex hand if it is different from eth0 (where ethX = correct ethernet interface) run
# (Although it may be simpler to connect the ReFlex to eth0 and communicat with motoman
# via eth1)
#
# rosparam set driver_node/network_interface ethX 
#

# ReFlex Hand finger and joint limits:
# 
# preshape (swivel): 0 to 1.57 (pi/2)
# fingers proximal: 0 to 2.88~
# fingers distal: 0 to 2.27~ (underactuated).

class ReFlexHand():
    def __init__(self, is_fake = False):

        self.fake_flag = is_fake
        if self.fake_flag:
            return

        # Starts up hand and publishes the command data
        self.reflex_hand = reflex_smarts.ReFlex_Smarts()
        rospy.sleep(1)

        # Zeroing the tactile data is necessary if you wish to use a mode
        # that employs tactile data (like guarded_move) because the
        # sensors drift over time. This creates a callable service proxy 
        zero_tactile = rospy.ServiceProxy('/zero_tactile', Empty)
        zero_tactile()
        rospy.sleep(2)

        # max open angle for inside bin
        self.maxOpenAngle = 1.55

        # max joint angle
        self.maxJointAngle = 3.14

        # max spread angle
        self.maxPreshapeAngle = 1.57

        # Setup grasp modes
        self.GM_basic = 0     # ?? - assume cylindrical
        self.GM_pinch = 1     # pinch
        self.GM_scissor = 2   # equivallent to cylindrical 
        self.GM_wide = 3      # ?? - assume = spherical - fingers equidistant
        self.GM_pinch2 = 4    # a special kind of pinch with thumb tucked in
        self.GM_custom = 5    # allow custom preshape angle

        self.grasping_modes = [self.GM_basic, self.GM_pinch, self.GM_scissor, self.GM_wide, self.GM_pinch2, self.GM_custom]
        self.setGraspMode(self.GM_basic)
        rospy.sleep(2)


    # set grasp mode
    def setGraspMode(self, mode, preshapeAngle=0, finger1Angle=0, finger2Angle=0, finger3Angle=0):
        if self.fake_flag:
            return
        if mode == self.GM_basic:
            # ?? - assume this is cylindrical grasp mode
            #self.reflex_hand.set_cylindrical(1) # doesn't work
            self.reflex_hand.move_preshape(0.0)
            rospy.sleep(1)
            self.open("bin_open")
        elif mode == self.GM_pinch:
            # pinch mode - 2 fingers opposite each other. thumb not used
            #self.reflex_hand.set_pinch(1) # doesn't work
            self.reflex_hand.move_preshape(self.maxPreshapeAngle)
            rospy.sleep(1)
            self.open("bin_open")
        elif mode == self.GM_scissor:
            # ?? - assume this is cylindrical grasp mode
            #self.reflex_hand.set_cylindrical(1) # doesn't work
            self.reflex_hand.move_preshape(0.0)
            rospy.sleep(1)
            self.open("bin_open")
        elif mode == self.GM_wide:
            # ?? - assume this is spherical grasp mode - fingers at 120 degrees from each other
            #self.reflex_hand.set_spherical(1) # doesn't work
            self.reflex_hand.move_preshape(1.05)
            rospy.sleep(1)
            self.open("bin_open")
        elif mode == self.GM_pinch2:
            #self.reflex_hand.set_pinch(1) doesn't work
            self.reflex_hand.move_preshape(self.maxPreshapeAngle)
            rospy.sleep(1)
            self.setJointAngles(self.maxOpenAngle, self.maxOpenAngle, self.maxJointAngle)
        elif mode == self.GM_custom:
            # custom finger preshape position
            self.reflex_hand.move_preshape(preshapeAngle)
            rospy.sleep(1)
            self.setJointAngles(finger1Angle, finger2Angle, finger3Angle)
            
        rospy.sleep(1)
        self._current_grasping_mode = mode



    # set joint angles
    def setJointAngles(self, finger1, finger2, thumb):
        if self.fake_flag:
            return        
        self.reflex_hand.move_finger(0, finger1)
        self.reflex_hand.move_finger(1, finger2)
        self.reflex_hand.move_finger(2, thumb)
        #self.reflex_hand.command_smarts(1, "f0 " + str(finger1) + " f1 " + str(finger2) + " f2 " + str(thumb))
        rospy.sleep(1)


    # since with all fingers fully extended the hand will not fit inside the shelves
    # we set the fingers to about 80 degrees / 1.4 radians
    def open(self,open_type="order_open"):
        if self.fake_flag:
            return
        if open_type == "bin_open":
            self.setJointAngles(self.maxOpenAngle, self.maxOpenAngle, self.maxOpenAngle)
        if open_type == "order_open":
            #self.reflex_hand.command_smarts(1, "open")
            self.setJointAngles(0.0, 0.0, 0.0)
        rospy.sleep(1);


    def close(self):
        if self.fake_flag:
            return
#    def close(self, graspType="close"):
        # preset reflex movements:
        #guarded_move - each finger closes and stops when contact felt while others move on
        #solid_contact - keeps squeezing until tendon limits reached or both finger links sense contact (on each finger)
        #avoid_contact - opens finger until boundary if any contact felt
        #maintain_contact - if no contact, finger keeps closing
        #dither - maintain light pressure on object
        #hold - just hold at current position
        #align_all - sets finger positions to the average of their positions
        #close- close reflex hand using the command_smarts interface (closes fingers very quickly)

        # ----- copied these 3 lines from the robotiq code. not sure what they are for.
        #global g_reflex_state
        #if self.fake_flag:
        #    return
        
        #self.reflex_hand.command_smarts(1, graspType) # command smarts doesnt work
        self.setJointAngles(self.maxJointAngle, self.maxJointAngle, self.maxJointAngle);


    def reset(self):
        if self.fake_flag:
            return


    def calibrate(self):
        return







# ------ code to test the class 

# rospy.loginfo("Initializing hand instance")
# reflex = ReFlexHand()

# # Setup grasp modes
# GM_basic = 0     # ?? - assume cylindrical
# GM_pinch = 1     # pinch
# GM_scissor = 2   # equivallent to cylindrical 
# GM_wide = 3      # ?? - assume = spherical - fingers equidistant
# GM_pinch2 = 4    # a special kind of pinch with thumb tucked in
# GM_custom = 5    # allow custom preshape angle

# rospy.loginfo("Grasp Mode: Basic")
# reflex.setGraspMode(GM_basic)
# rospy.sleep(4)

# rospy.loginfo("Grasp Mode: Pinch")
# reflex.setGraspMode(GM_pinch)
# rospy.sleep(4)

# rospy.loginfo("Grasp Mode: Special Pinch")
# reflex.setGraspMode(GM_pinch2)
# rospy.sleep(4)

# rospy.loginfo("Grasp Mode: Scissor")
# reflex.setGraspMode(GM_scissor)
# rospy.sleep(4)

# rospy.loginfo("Grasp Mode: Wide")
# reflex.setGraspMode(GM_wide)
# rospy.sleep(4)

# rospy.loginfo("Grasp Mode: Custom")
# reflex.setGraspMode(GM_custom, 0.7, 1.7, 1.2, 0.5)
# rospy.sleep(4)

# rospy.loginfo("Order Open: Open all the way")
# reflex.open()
# rospy.sleep(4)

# rospy.loginfo("Closing Hand")
# reflex.setGraspMode(GM_basic)
# reflex.close()
# rospy.sleep(4)

## all of the following are supposed to work but for some reason the smarts command function seems to
## crash every time. These do however work if you run "rosservice call /reflex/command_smarts [cmd]" from
## the command line. I will follow up with RightHand Robotics to see what's wrong

#rospy.loginfo("Closing Hand: Guarded Move")
#reflex.open()
#reflex.close("guarded_move")
#rospy.sleep(10)

#rospy.loginfo("Closing Hand: Solid Contact")
#reflex.open()
#reflex.close("solid_contact")
#rospy.sleep(10)

#rospy.loginfo("Closing Hand: Maintain Contact")
#reflex.open()
#reflex.close("maintain_contact")
#rospy.sleep(10)

#rospy.loginfo("Closing Hand: Dither")
#reflex.open()
#reflex.close("dither")
#rospy.sleep(10)

#rospy.loginfo("Closing Hand: Hold")
#reflex.close("hold")
#rospy.sleep(10)

#rospy.spin()

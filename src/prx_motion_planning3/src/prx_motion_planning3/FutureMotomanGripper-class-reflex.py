from std_srvs.srv import Empty
 
# Imports the code to control the hand using smart actions
from reflex import reflex_smarts

# To set the correct ethernet interface used for communication with the 
# ReFlex hand if it is different from eth0 (where ethX = correct ethernet interface) run
# (Although it may be simpler to connect the ReFlex to eth0 and communicat with motoman
# via eth1)
#
# rosparam set driver_node/network_interface ethX 
#

class FutureMotomanGripper():
    def __init__(self):

        # Starts up hand and publishes the command data
        self.reflex_hand = reflex_smarts.ReFlex_Smarts()
        rospy.sleep(1)

        # Zeroing the tactile data is necessary if you wish to use a mode
        # that employs tactile data (like guarded_move) because the
        # sensors drift over time. This creates a callable service proxy 
        #zero_tactile = rospy.ServiceProxy('/zero_tactile', Empty)
        #zero_tactile()
        #rospy.sleep(2)

        # max open angle for inside bin
        self.maxOpenAngle = 1.4;

        # Setup grasp modes
        self.GM_basic = 0     # ?? - assume cylindrical
        self.GM_pinch = 1     # pinch
        self.GM_scissor = 2   # equivallent to cylindrical 
        self.GM_wide = 3      # ?? - assume = spherical - fingers equidistant
        self.GM_custom = 4    # allow custom preshape angle

        self.grasping_modes = [self.GM_basic, self.GM_pinch, self.GM_scissor, self.GM_wide, self.GM_custom]
        self.setGraspMode(self.GM_basic)


    # set grasp mode
    def setGraspMode(self, mode, preshapeAngle=0):
        if mode == self.GM_basic:
            # ?? - assume this is cylindrical grasp mode
            self.reflex_hand.set_cylindrical(1)
            rospy.sleep(2)
        elif mode == self.GM_pinch:
            # pinch mode - 2 fingers opposite each other. thumb not used
            self.reflex_hand.set_pinch(1)
            rospy.sleep(2)
        elif mode == self.GM_scissor:
            # cylindrical grasp mode 
            self.reflex_hand.set_cylindrical(1)
            rospy.sleep(2)
        elif mode == self.GM_wide:
            # ?? - assume this is spherical grasp mode - fingers at 120 degrees from each other
            self.reflex_hand.set_spherical(1)
            rospy.sleep(2)
        elif mode == self.GM_custom:
            # custom finger preshape position
            self.reflex_hand.move_preshape(preshapeAngle)
            rospy.sleep(2)
            
        self._current_grasping_mode = mode
        self.open("bin_open")


    # set joint angles
    def setJointAngles(self, finger1, finger2, thumb):
        self.reflex_hand.move_finger(0, finger1)
        self.reflex_hand.move_finger(1, finger2)
        self.reflex_hand.move_finger(2, thumb)
        rospy.sleep(2)


    # since with all fingers fully extended the hand will not fit inside the shelves
    # we set the fingers to about 80 degrees / 1.4 radians
    def open(self,open_type=""):

        if open_type == "bin_open":
            self.setJointAngles(self.maxOpenAngle, self.maxOpenAngle, self.maxOpenAngle)
        if open_type == "order_open":
            self.reflex_hand.command_smarts(1, "open")
            rospy.sleep(2);


    def close(self):
        global g_reflex_state
        if self.fake_flag:
            return
        
        # close reflex hand using the command_smarts interface 
        # (closes fingers very quickly)
        reflex_hand.command_smarts(1, 'close')

        # close reflex hand by commanding each finger to move slowly and carefully
        # coming soon


    def reset(self):
        if self.fake_flag:
            return


    def calibrate(self):
        return

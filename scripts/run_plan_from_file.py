import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_core_msgs.msg import JointCommand
import rospy
import sys
import time


def is_grasp(gripper_state):
    if(gripper_state==1):
        return True
    else:
        return False


def read_file(file):
    global joint_command_publisher
    global g_limb
    global g_left_gripper
    global g_endpoint_state

    f=open(file,"r")
    lines=f.readlines()
    RATE=0.1
    cmd=JointCommand()
    cmd.mode=1
    g_left_gripper.open()
    names=["left_s0","left_s1","left_e0","left_e1","left_w0","left_w1","left_w2"]
    cmd.names=names
    print cmd
    joint_str=lines[0].rstrip().split(',')
    joints=[float(joint) for joint in joint_str]
    limb_pos=dict(zip(names,joints[0:7]))
    is_grasping=is_grasp(joints[7])
    
    if(is_grasping):
        g_left_gripper.close()
    else:
        g_left_gripper.open()

    g_limb.move_to_joint_positions(limb_pos)
    
    i=1
    
    while i<len(lines) and not rospy.is_shutdown():
        line=lines[i]
        
        joint_str=line.rstrip().split(',')
        joints=[float(joint) for joint in joint_str]
        if is_grasping != is_grasp(joints[7]):
            if is_grasp(joints[7]):
                
                g_left_gripper.close()
                is_grasping=True
            else:
                g_left_gripper.open()

        cmd.command=joints[0:7]
        # print joints[0:7]
        print i
        joint_command_publisher.publish(cmd)
        time.sleep(RATE)
        
        i=i+1


def read_file_main(path_file_name):
    global g_limb
    global g_left_gripper
    global joint_command_publisher

    rospy.init_node('run_file', anonymous=True)
    
    # Enable robot
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    rs.enable()

    g_limb = baxter_interface.Limb('left')
    g_left_gripper = baxter_interface.Gripper('left')
    if g_left_gripper.error():
        g_left_gripper.reset()
    if (not g_left_gripper.calibrated() and g_left_gripper.type() != 'custom'):
        g_left_gripper.calibrate()
    joint_command_publisher = rospy.Publisher("/robot/limb/left/joint_command",JointCommand, queue_size=1,tcp_nodelay=True)
    read_file(path_file_name)
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    read_file_main(sys.argv[1])



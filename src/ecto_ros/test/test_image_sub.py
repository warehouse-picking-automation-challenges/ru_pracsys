#!/usr/bin/env python
import ecto
import ecto_ros
import ecto_ros.ecto_sensor_msgs as ecto_sensor_msgs
from ecto_ros_test_utils import *

ImageSub = ecto_sensor_msgs.Subscriber_Image

def do_ecto(bagname, msg_counts, Scheduler):
    sub_rgb = ImageSub("image_sub", topic_name='/camera/rgb/image_color', queue_size=0)
    sub_depth = ImageSub("depth_sub", topic_name='/camera/depth/image', queue_size=0)
    
    im2mat_rgb = ecto_ros.Image2Mat()
    im2mat_depth = ecto_ros.Image2Mat()
    counter_rgb = ecto.Counter(every=10)
    counter_depth = ecto.Counter()
    graph = [
                sub_rgb["output"] >> im2mat_rgb["image"],
                sub_depth["output"] >> im2mat_depth["image"],
                im2mat_rgb[:] >> counter_rgb[:],
                im2mat_depth[:] >> counter_depth[:]
            ]
    plasm = ecto.Plasm()
    plasm.connect(graph)
    sched = Scheduler(plasm)
    sched.execute_async()
    rosbag = play_bag(bagname, delay=1, rate=0.5)#rate hack for fidelity in message count FIXME
    wait_bag(rosbag)
    time.sleep(0.1)
    sched.stop()

    print "expecting RGB count:", msg_counts['/camera/rgb/image_color']
    print "RGB count:", counter_rgb.outputs.count
    print "expecting Depth count:", msg_counts['/camera/depth/image']
    print "Depth count:", counter_depth.outputs.count
    assert msg_counts['/camera/rgb/image_color'] >= counter_rgb.outputs.count
    assert msg_counts['/camera/rgb/image_color'] >= counter_depth.outputs.count
    assert counter_rgb.outputs.count != 0
    assert counter_depth.outputs.count != 0
    
if __name__ == "__main__":
    bagname = sys.argv[1]
    msg_counts = bag_counts(bagname)
    try:
        roscore = start_roscore(delay=1)
        ecto_ros.init(sys.argv, "image_sub_node")
        do_ecto(bagname, msg_counts, ecto.schedulers.Singlethreaded)
    finally:
        roscore.terminate()

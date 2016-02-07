#!/usr/bin/env python
import ecto
import ecto_ros, ecto_ros.ecto_sensor_msgs as ecto_sensor_msgs
import sys

ImageSub = ecto_sensor_msgs.Subscriber_Image
ImagePub = ecto_sensor_msgs.Publisher_Image

def do_ecto():
    sub_rgb = ImageSub("image_sub",topic_name='/camera/rgb/image_mono')
    sub_depth = ImageSub("depth_sub",topic_name='/camera/depth/image')
    pub_rgb = ImagePub("image_pub", topic_name='my_image')
    pub_depth = ImagePub("depth_pub", topic_name='my_depth')
    
    graph = [
                sub_rgb["output"] >> pub_rgb["input"],
                sub_depth["output"] >> pub_depth["input"]
            ]
    
    plasm = ecto.Plasm()
    plasm.connect(graph)
    
    ecto.view_plasm(plasm)
    
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute()
    
if __name__ == "__main__":
    ecto_ros.init(sys.argv, "image_pub")
    do_ecto()

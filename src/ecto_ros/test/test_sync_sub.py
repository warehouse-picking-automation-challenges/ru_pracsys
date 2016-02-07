#!/usr/bin/env python
from ecto_opencv import highgui
from ecto_ros_test_utils import *
import ecto
from ecto_ros import Synchronizer
import ecto_ros.ecto_sensor_msgs as ecto_sensor_msgs
import sys

ImageSub = ecto_sensor_msgs.Subscriber_Image
CameraInfoSub = ecto_sensor_msgs.Subscriber_CameraInfo

subs = dict(
            image=ImageSub(topic_name='/camera/rgb/image_color', queue_size=0),
            depth=ImageSub(topic_name='/camera/depth/image', queue_size=0),
            depth_info=CameraInfoSub(topic_name='/camera/depth/camera_info', queue_size=0),
            image_info=CameraInfoSub(topic_name='/camera/rgb/camera_info', queue_size=0),
         )

sync = Synchronizer('Synchronizator', subs=subs
                             )
counter_rgb = ecto.Counter()
counter_depth = ecto.Counter()
counter_rgb_info = ecto.Counter()
counter_depth_info = ecto.Counter()

graph = [
            sync["image"] >> counter_rgb[:],
            sync["depth"] >> counter_depth[:],
            sync["image_info"] >> counter_rgb_info[:],
            sync["depth_info"] >> counter_depth_info[:],
        ]
plasm = ecto.Plasm()
plasm.connect(graph)

#!/usr/bin/env python
import ecto
import ecto_ros, ecto_ros.ecto_sensor_msgs as ecto_sensor_msgs
from ecto_opencv import highgui
import sys
import argparse

ImagePub = ecto_sensor_msgs.Publisher_Image

def do_ecto(device_id=0, frame_id='base'):
    video_capture = highgui.VideoCapture('Video Camera', video_device=device_id)
    mat2image = ecto_ros.Mat2Image(frame_id=frame_id, encoding='bgr8')
    pub_rgb = ImagePub("image pub", topic_name='image')
    
    graph = [
                video_capture["image"] >> mat2image["image"],
                mat2image["image"] >> pub_rgb["input"]
            ]
    
    plasm = ecto.Plasm()
    plasm.connect(graph)
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute()
    
if __name__ == "__main__":
    ecto_ros.init(sys.argv, "image_pub")
    
    parser = argparse.ArgumentParser(description='Turns a /dev/video* into a ros image publisher.')
    parser.add_argument('--device_id', metavar='N', dest='device_id', type=int, default=0,
                       help='the device id to open.')
    parser.add_argument('--frame_id', dest='frame_id', type=str, default='base',
                       help='The frame id to associate this camera with.')
    args = parser.parse_args()
    
    do_ecto(args.device_id, args.frame_id)

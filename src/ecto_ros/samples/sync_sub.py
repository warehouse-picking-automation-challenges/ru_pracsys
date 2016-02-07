#!/usr/bin/env python
import ecto
import ecto_ros, ecto_ros.ecto_sensor_msgs as ecto_sensor_msgs
from ecto_opencv.highgui import imshow

ImageSub = ecto_sensor_msgs.Subscriber_Image

def do_ecto():
    #Set up the ecto_ros subscribers, with a dict of output name -> subscriber cell
    subs = dict(image=ImageSub(topic_name='image',queue_size=0),
                depth=ImageSub(topic_name='depth',queue_size=0),
                )
    #The synchronizer expects this dict, and will have an output foreach key:value pair
    sync = ecto_ros.Synchronizer('Synchronizator', subs=subs)

    #some rosmsg ~> opencv type converters
    rgb = ecto_ros.Image2Mat()
    depth = ecto_ros.Image2Mat()
    
    #declare a graph that just shows the images using opencv highgui
    graph = [
                sync["image"] >> rgb["image"],
                sync["depth"] >> depth["image"],
                rgb["image"] >> imshow(name="rgb")["image"],
                depth["image"] >> imshow(name="depth")["image"]
            ]
    
    #add the graph to our ecto plasm
    plasm = ecto.Plasm()
    plasm.connect(graph)
    
    #We'll use a threadpool based scheduler to execute this one.
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute() #execute forever

if __name__ == "__main__":
    import sys
    ecto_ros.init(sys.argv, "ecto_node") #strips out ros remappings and initializes roscpp
    do_ecto()

/* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Robotiq, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Robotiq, Inc. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Copyright (c) 2014, Robotiq, Inc
*/

/**
 * \file rq_test_sensor.cpp
 * \date July 14, 2014
 *  \author Jonathan Savoie <jonathan.savoie@robotiq.com>
 *  \maintainer Nicolas Lauzier <nicolas@robotiq.com>
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "robotiq_force_torque_sensor/ft_sensor.h"
#include "robotiq_force_torque_sensor/sensor_accessor.h"
#include "robotiq_force_torque_sensor/ft_feedback.h"
#include <prx_decision_making/DecisionMakingStateMessage.h>
#include <prx_decision_making/decision_making.h>

#include <actionlib_msgs/GoalID.h>

#include <sstream>
#include <sys/time.h>

/*void receiveCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}*/
#define PREEMPT_TIMEOUT 10

ros::Publisher cancel_publisher;
ros::Publisher feedback_publisher;

/** @brief When the timer was started */
struct timeval start;
/** @brief When the timer finished */
struct timeval finish;

int g_robot_arm;

//callback for leftSensor
void leftSensorCallback(const robotiq_force_torque_sensor::ft_sensor& msg)
{
  // ROS_INFO("I heard: FX[%f] FY[%f] FZ[%f] MX[%f] MY[%f] MZ[%f] Rf[%f]", msg.Fx,msg.Fy,msg.Fz,msg.Mx,msg.My,msg.Mz,msg.Rf);
  //only work if left arm is working
  if(g_robot_arm == LEFT_ARM)
  {
    if(msg.isHeavy > 0)
    {
      gettimeofday( &finish, NULL);
      double elapsed = (finish.tv_sec - start.tv_sec) + 0.000001 * (finish.tv_usec - start.tv_usec);
      if(elapsed > PREEMPT_TIMEOUT)
      {
        gettimeofday( &start, NULL);
        ROS_INFO("Preempting...");
        actionlib_msgs::GoalID empty_msg;
        cancel_publisher.publish(empty_msg);
        robotiq_force_torque_sensor::ft_feedback feedback;
        feedback.preempted = true;
        feedback_publisher.publish(feedback);
      }
    }  
  }
  
}

//callback for rightSensor
void rightSensorCallback(const robotiq_force_torque_sensor::ft_sensor& msg)
{
  // ROS_INFO("I heard: FX[%f] FY[%f] FZ[%f] MX[%f] MY[%f] MZ[%f] Rf[%f]", msg.Fx,msg.Fy,msg.Fz,msg.Mx,msg.My,msg.Mz,msg.Rf);
  //only work if right arm in motion
  if(g_robot_arm == RIGHT_ARM)
  {
    if(msg.isHeavy > 0)
    {
      gettimeofday( &finish, NULL);
      double elapsed = (finish.tv_sec - start.tv_sec) + 0.000001 * (finish.tv_usec - start.tv_usec);
      if(elapsed > PREEMPT_TIMEOUT)
      {
        gettimeofday( &start, NULL);
        ROS_INFO("Preempting...");
        actionlib_msgs::GoalID empty_msg;
        cancel_publisher.publish(empty_msg);
        robotiq_force_torque_sensor::ft_feedback feedback;
        feedback.preempted = true;
        feedback_publisher.publish(feedback);
      }
    }  
  }
}

//callback to decide left/right arm
void ft_arm_handler(prx_decision_making::DecisionMakingStateMessagePtr system_state)
{
  g_robot_arm = system_state->robot_arm;
}




/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  gettimeofday( &start, NULL ); 
  gettimeofday( &finish, NULL);
  ros::init(argc, argv, "rq_test_sensor");


  ros::NodeHandle n;
  //left
  ros::ServiceClient leftclient = n.serviceClient<robotiq_force_torque_sensor::sensor_accessor>("robotiq_force_torque_sensor_acc_left");
  ros::Subscriber leftsub1 = n.subscribe("robotiq_force_torque_sensor_left",100,leftSensorCallback);
  //right
  ros::ServiceClient rightclient = n.serviceClient<robotiq_force_torque_sensor::sensor_accessor>("robotiq_force_torque_sensor_acc_right");
  ros::Subscriber rightsub1 = n.subscribe("robotiq_force_torque_sensor_right",100,rightSensorCallback);

  //handler for detecting which arm is in motion
  ros::Subscriber decision_making_sub = n.subscribe("/decision_making_state", 1, ft_arm_handler);
  
  cancel_publisher = n.advertise<actionlib_msgs::GoalID>("/joint_trajectory_action/cancel", 1);
  feedback_publisher = n.advertise<robotiq_force_torque_sensor::ft_feedback>("ft_feedback", 1);

  robotiq_force_torque_sensor::sensor_accessor srv;

  srv.request.command = "SET ZRO";
  if(leftclient.call(srv))
  {
   ROS_INFO("ret: %s", srv.response.res.c_str());
  }
  if(rightclient.call(srv))
  {
   ROS_INFO("ret: %s", srv.response.res.c_str());
  }
  int count = 0;
  while (ros::ok())
  {
    // if(count == 10000000)
   //  {
    //  srv.request.command = "SET ZRO";
    //  if(client.call(srv))
   //    {
    //    ROS_INFO("ret: %s", srv.response.res.c_str());
    //  }
    //  count = 0;
    // }  
    // if(count == 100000000)
    // {
      
    //   ROS_INFO("Hello from rq_test_sensor");
    //   actionlib_msgs::GoalID empty_msg;
    //   cancel_publisher.publish(empty_msg);
    //   count = 0;
    // }

    ros::spinOnce();

    ++count;
  }


  return 0;
}

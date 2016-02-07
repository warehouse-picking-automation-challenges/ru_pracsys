/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once
#include <ecto/ecto.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <string>

namespace ecto_ros
{
  /**
   * \brief Use this to wrap a simple ros message subscriber.
   */
  template<typename MessageT>
  struct Publisher
  {
    typedef typename MessageT::ConstPtr MessageConstPtr;
    //ros subscription stuffs
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    std::string topic_;
    int queue_size_;
    bool latched_;
    ecto::spore<MessageConstPtr> in_;
    ecto::spore<bool> has_subscribers_;

    void
    setupPubs()
    {
      //look up remapping
      std::string topic = nh_.resolveName(topic_, true);
      pub_ = nh_.advertise<MessageT>(topic, queue_size_, latched_);
      ROS_INFO_STREAM("publishing to topic:" << topic);
    }

    static void
    declare_params(ecto::tendrils& p)
    {
      p.declare<std::string>("topic_name",
                             "The topic name to publish to. May be remapped.",
                             "/ros/topic/name").required(true);
      p.declare<int>("queue_size", "The amount to buffer incoming messages.", 2);
      p.declare<bool>("latched", "Is this a latched topic?", false);

    }

    static void
    declare_io(const ecto::tendrils& /*p*/, ecto::tendrils& in, ecto::tendrils& out)
    {
      in.declare<MessageConstPtr>("input", "The message to publish.").required(true);
      out.declare<bool>("has_subscribers", "Has currently connected subscribers.");
    }

    void
    configure(const ecto::tendrils& p, const ecto::tendrils& in, const ecto::tendrils& out)
    {
      topic_ = p.get<std::string>("topic_name");
      queue_size_ = p.get<int>("queue_size");
      latched_ = p.get<bool>("latched");
      in_ = in["input"];
      has_subscribers_ = out["has_subscribers"];
      // initialise this so it can be used by an entangled pair to avoid work constructing
      // the messages before publishing (lazy publishing!)
      *has_subscribers_ = false;
      setupPubs();
    }

    int
    process(const ecto::tendrils& in, const ecto::tendrils& out)
    {
      int num_subscribers = pub_.getNumSubscribers();
      *has_subscribers_ = (num_subscribers != 0) ? true : false;
      // lazy publishing if appropriate conditions are met
      if(*in_ && (*has_subscribers_ || latched_)) { 
        pub_.publish(**in_);
      }
      return ecto::OK;
    }
  };
}

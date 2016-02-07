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
#include <rosbag/view.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <string>

namespace ecto_ros
{
   /**
   * \brief Use this to wrap a simple ros message subscriber.
   */
  template<typename MessageT>
  struct Subscriber
  {
    typedef typename MessageT::ConstPtr MessageConstPtr;
    typedef typename MessageT::Ptr MessagePtr;

    //ros subscription stuffs
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::string topic_;
    size_t queue_size_;
    boost::condition_variable cond_;
    boost::mutex mut_;
    ecto::spore<MessageConstPtr> out_;
    boost::thread sub_thread_;
    std::list<MessageConstPtr> queue_;

    void
    setupSubs()
    {
      //look up remapping
      std::string topic = nh_.resolveName(topic_, true);
      sub_ = nh_.subscribe(topic, queue_size_, &Subscriber::dataCallback, this);
      ROS_INFO_STREAM("Subscribed to topic:" << topic << " with queue size of " << queue_size_);
    }

    void
    setupSubs_async()
    {
      sub_thread_ = boost::thread(boost::bind(&Subscriber<MessageT>::setupSubs,this));
    }

    void
    dataCallback(const typename MessageT::ConstPtr& data)
    {
      //use condition variable to allow asynchronous
      //callback as this callback may or may not happen on
      //the same thread as the process function.
      {
        boost::lock_guard<boost::mutex> lock(mut_);
        queue_.push_back(data);
        if (queue_.size() > queue_size_)
          queue_.pop_front();
      }
      cond_.notify_one();
    }

    static void
    declare_params(ecto::tendrils& params)
    {
      params.declare<std::string>("topic_name", "The topic name to subscribe to.", "/ros/topic/name").required(true);
      params.declare<int>("queue_size", "The amount to buffer incoming messages.", 2);
    }

    static void
    declare_io(const ecto::tendrils& p, ecto::tendrils& in, ecto::tendrils& out)
    {
      out.declare<MessageConstPtr>("output", "The received message.");
    }

    void
    configure(const ecto::tendrils& p, const ecto::tendrils& in, const ecto::tendrils& out)
    {
      topic_ = p.get<std::string>("topic_name");
      queue_size_ = p.get<int>("queue_size");
      out_ = out["output"];
      setupSubs_async();
    }

    int
    process(const ecto::tendrils& in, const ecto::tendrils& out)
    {
      sub_thread_.join();
      //condition variable idiom, blocks until the data has been filled by ros.
      unsigned int counter = 0;
      boost::unique_lock<boost::mutex> lock(mut_);
      while (queue_.empty())
      {
        boost::this_thread::interruption_point();
        cond_.timed_wait(lock,boost::posix_time::millisec(5));
        if ( counter++ > 40 )  {
          return ecto::DO_OVER; // give the scheduler a chance to manage (e.g. abort)
        }
      }
      *out_ = queue_.front();
      queue_.pop_front();
      //reset the data_ so that the condition variable still works.
      return ecto::OK;
    }
  };
}

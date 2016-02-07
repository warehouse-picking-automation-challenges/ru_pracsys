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
#include <rosbag/view.h>
#include <ros/time.h>
#include <string>

namespace ecto_ros
{
  struct Bagger_base
  {
    typedef boost::shared_ptr<const Bagger_base> ptr;
    virtual
    ~Bagger_base()
    {
    }
    virtual ecto::tendril_ptr
    instantiate() const = 0;
    virtual ecto::tendril_ptr
    instantiate(rosbag::View::iterator message) const = 0;

    virtual
    void
    write(rosbag::Bag& bag, const std::string& topic, const ros::Time& stamp, const ecto::tendril& t) const = 0;

  };

  /**
   * \brief enables instantiation from a bag.
   */
  template<typename MessageT>
  struct Bagger: Bagger_base
  {
    typedef typename MessageT::ConstPtr MessageConstPtr;

    virtual
    ~Bagger()
    {
    }

    ecto::tendril_ptr
    instantiate() const
    {
      ecto::tendril_ptr tp = ecto::make_tendril<MessageConstPtr>();
      return tp;
    }
    ecto::tendril_ptr
    instantiate(rosbag::View::iterator message) const
    {
      ecto::tendril_ptr tp = instantiate();
      MessageConstPtr mcp = message->instantiate<MessageT>();
      if (mcp)
        tp << mcp;
      return tp;
    }

    void
    write(rosbag::Bag& bag, const std::string& topic, const ros::Time& stamp, const ecto::tendril& t) const
    {
      MessageConstPtr mcp;
      t >> mcp;
      bag.write(topic, stamp, *mcp);
    }

    static void
    declare_params(ecto::tendrils& params)
    {
      params.declare<std::string>("topic_name", "The topic name to subscribe to.", "/ros/topic/name").required(true);
      params.declare<Bagger_base::ptr>("bagger", "The bagger.", Bagger_base::ptr(new Bagger<MessageT>()));
    }

  };
}

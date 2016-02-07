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

#include <ecto/ecto.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <iostream>
#include <string>

namespace ecto_ros
{

  using ecto::tendrils;
  using std::string;
  using namespace sensor_msgs;

  struct DriftPrinter
  {

    static void
    declare_io(const tendrils& /*p*/, tendrils& i, tendrils& o)
    {
      i.declare<ImageConstPtr> ("image").required(true);
      i.declare<CameraInfoConstPtr> ("image_info").required(true);
      i.declare<ImageConstPtr> ("depth").required(true);
      i.declare<CameraInfoConstPtr> ("depth_info").required(true);
    }

    static double
    calcOffset(ros::Time rhs, ros::Time lhs)
    {
      return std::abs(rhs.toSec() - lhs.toSec()) * 1000;
    }
    int
    process(const tendrils& i, const tendrils& o)
    {
      ImageConstPtr image, depth;
      CameraInfoConstPtr image_info, depth_info;
      i["image"] >> image;
      i["image_info"] >> image_info;
      i["depth"] >> depth;
      i["depth_info"] >> depth_info;

      std::cout << "\ndrifts(millis):\n\t" << "image -> image_info " << calcOffset(image_info->header.stamp,
                                                                                   image->header.stamp)
          << "\n\timage -> depth " << calcOffset(image_info->header.stamp, depth->header.stamp)
          << "\n\tdepth -> depth_info " << calcOffset(depth->header.stamp, depth_info->header.stamp);

      return ecto::OK;
    }
  };

}

ECTO_CELL(ecto_ros_main, ecto_ros::DriftPrinter, "DriftPrinter", "Prints timing drift. For image,depth,camera_infos.")


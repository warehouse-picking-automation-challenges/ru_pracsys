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

#include <geometry_msgs/PoseStamped.h>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>

#include <iostream>
#include <string>

namespace ecto_ros
{

  using ecto::tendrils;
  using std::string;
  using namespace geometry_msgs;

  struct RT2PoseStamped
  {

    static void
    declare_params(tendrils& p)
    {
      p.declare<std::string>("frame_id", "The frame id that generated the pose.").required(true);
    }
    static void
    declare_io(const tendrils& /*p*/, tendrils& i, tendrils& o)
    {
      i.declare<cv::Mat>("R", "3X3 Rotation matrix.");
      i.declare<cv::Mat>("T", "3X1 Translation vector.");
      o.declare<PoseStampedConstPtr>("pose", "A geometry_msgs::PoseStamped.");
    }
    void
    configure(const tendrils& p, const tendrils& i, const tendrils& o)
    {
      ros::Time::init();
      R_ = i["R"];
      T_ = i["T"];
      pose_ = o["pose"];
      frame_id_ = p["frame_id"];
    }
    int
    process(const tendrils&, const tendrils&)
    {
      wpose_.reset(new PoseStamped);
      *pose_ = wpose_;
      if (R_->empty() || T_->empty())
        return ecto::OK;
      cv::Mat R, T;
      R_->convertTo(R, CV_32F);
      T_->convertTo(T, CV_32F);
      Eigen::Matrix3f rotation_matrix;
      for (unsigned int j = 0; j < 3; ++j)
        for (unsigned int i = 0; i < 3; ++i)
          rotation_matrix(j, i) = R.at<float>(j, i);

      Eigen::Quaternion<float> quaternion(rotation_matrix);

      PoseStamped& pose = *wpose_;
      pose.pose.position.x = T.at<float>(0);
      pose.pose.position.y = T.at<float>(1);
      pose.pose.position.z = T.at<float>(2);
      pose.pose.orientation.x = quaternion.x();
      pose.pose.orientation.y = quaternion.y();
      pose.pose.orientation.z = quaternion.z();
      pose.pose.orientation.w = quaternion.w();
      pose.header.seq++;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = *frame_id_;
      return ecto::OK;
    }
    PoseStampedPtr wpose_;
    ecto::spore<std::string> frame_id_;
    ecto::spore<PoseStampedConstPtr> pose_;
    ecto::spore<cv::Mat> R_, T_;
  };

  struct PoseStamped2RT
  {
    static void
    declare_io(const tendrils& /*p*/, tendrils& i, tendrils& o)
    {
      i.declare<PoseStampedConstPtr>("pose", "A geometry_msgs::PoseStamped.");
      o.declare<cv::Mat>("R", "3X3 Rotation matrix.");
      o.declare<cv::Mat>("T", "3X1 Translation vector.");
      o.declare<std::string>("frame_id", "The frame id of the pose.");
    }
    void
    configure(const tendrils& p, const tendrils& i, const tendrils& o)
    {
      pose_ = i["pose"];
      R_ = o["R"];
      T_ = o["T"];
      frame_id_ = o["frame_id"];
    }
    int
    process(const tendrils&, const tendrils&)
    {
      wpose_ = *pose_;
      cv::Mat_<double> R(3, 3), T(3, 1);
      geometry_msgs::Quaternion q = wpose_->pose.orientation;
      Eigen::Quaternion<float> quaternion(q.w, q.x, q.y, q.z); //w first !!

      Eigen::Matrix3f rotation_matrix(quaternion.matrix());
      for (unsigned int j = 0; j < 3; ++j)
        for (unsigned int i = 0; i < 3; ++i)
          R(j, i) = rotation_matrix(j, i);

      geometry_msgs::Pose::_position_type position = wpose_->pose.position;
      T(0) = position.x;
      T(1) = position.y;
      T(2) = position.z;
      *R_ = R;
      *T_ = T;
      return ecto::OK;
    }
    PoseStampedConstPtr wpose_;
    ecto::spore<std::string> frame_id_;
    ecto::spore<PoseStampedConstPtr> pose_;
    ecto::spore<cv::Mat> R_, T_;
  };
}

ECTO_CELL(ecto_ros_main, ecto_ros::RT2PoseStamped, "RT2PoseStamped",
          "Takes an R and T cv::Mat style and emits a stamped pose.")
ECTO_CELL(ecto_ros_main, ecto_ros::PoseStamped2RT, "PoseStamped2RT",
          "Takes a geometry_msgs::PoseStamped and turn it into a cv::Mat R and T.")

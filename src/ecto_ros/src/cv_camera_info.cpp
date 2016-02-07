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
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <string>
#include <ros/ros.h>

namespace ecto_ros
{

  using ecto::tendrils;
  using std::string;
  using namespace sensor_msgs;

  struct CameraInfo2Cv
  {
    static void
    declare_io(const tendrils& /*p*/, tendrils& i, tendrils& o)
    {
      i.declare(&CameraInfo2Cv::camera_info_, "camera_info");

      o.declare(&CameraInfo2Cv::K_, "K");
      o.declare(&CameraInfo2Cv::D_, "D");
      o.declare(&CameraInfo2Cv::image_size_,"image_size");
    }
    int
    process(const tendrils&, const tendrils&)
    {
      CameraInfo ci = **camera_info_;
      *K_ = cv::Mat(3, 3, CV_32FC1);
      for (int i = 0; i < 9; i++)
        K_->at<float>(i / 3, i % 3) = ci.K[i];
      *D_ = cv::Mat(ci.D.size(), 1, CV_32FC1);
      for (size_t i = 0; i < ci.D.size(); i++)
        D_->at<float>(i) = ci.D[i];
      *image_size_ = cv::Size(ci.width, ci.height);
      return ecto::OK;
    }
    ecto::spore<CameraInfoConstPtr> camera_info_;
    ecto::spore<cv::Mat> K_, D_;
    ecto::spore<cv::Size> image_size_;
  };

  struct Cv2CameraInfo
  {
    static void
    declare_params(tendrils& p)
    {
      p.declare<std::string>("frame_id");
    }
    static void
    declare_io(const tendrils& p, tendrils& i, tendrils& o)
    {
      i.declare<cv::Mat>("K");
      i.declare<cv::Mat>("D");
      i.declare<cv::Size>("image_size");
      o.declare<CameraInfoConstPtr>("camera_info");
    }
    void
    configure(const tendrils& p, const tendrils& i, const tendrils& o)
    {
      frame_id_ = p["frame_id"];
      K_ = i["K"];
      D_ = i["D"];
      image_size_ = i["image_size"];
      camera_info_ = o["camera_info"];
    }
    int
    process(const tendrils&, const tendrils&)
    {
      CameraInfoPtr ci(new CameraInfo);
      ci->header.frame_id = *frame_id_;
      ci->header.seq++;
      if (ros::isInitialized())
      {
        ci->header.stamp = ros::Time::now();
      }
      else
      {
        ros::WallTime w = ros::WallTime::now();
        ci->header.stamp = ros::Time(w.sec, w.nsec);
      }
      cv::Mat K;
      K_->convertTo(K, CV_64F); //convert to double
      for (int i = 0; i < 9; i++)
        ci->K[i] = K.at<double>(i / 3, i % 3);

      cv::Mat D;
      D_->convertTo(D, CV_64F);

      if (!D.empty())
      {
        ci->D.resize(D.rows);
        for (int i = 0; i < D.rows; i++)
        {
          ci->D[i] = D.at<double>(0, i);
        }
      }
      ci->R[0] = ci->R[4] = ci->R[8] = 1;
      ci->P[0] = ci->P[5] = ci->P[9] = 1;

      ci->width = image_size_->width;
      ci->height = image_size_->height;
      *camera_info_ = ci;
      return ecto::OK;
    }
    ecto::spore<CameraInfoConstPtr> camera_info_;
    ecto::spore<cv::Mat> K_, D_;
    ecto::spore<cv::Size> image_size_;
    ecto::spore<std::string> frame_id_;
  };
}
ECTO_CELL(ecto_ros_main, ecto_ros::CameraInfo2Cv, "CameraInfo2Cv",
          "Takes a CameraInfo message and converts to OpenCV types.")
ECTO_CELL(ecto_ros_main, ecto_ros::Cv2CameraInfo, "Cv2CameraInfo",
          "Takes opencv style camera info, and converts to an CameraInfo message.")

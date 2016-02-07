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
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <string>

#include <ros/ros.h>

namespace
{
  void
  header_stanza(std_msgs::Header& header)
  {
    header.seq++;
    if (ros::isInitialized())
    {
      header.stamp = ros::Time::now();
    }
    else
    {
      ros::WallTime w = ros::WallTime::now();
      header.stamp = ros::Time(w.sec, w.nsec);
    }
  }
  namespace enc = sensor_msgs::image_encodings;
  int
  getCvType(const std::string& encoding)
  {
    // Check for the most common encodings first
    if (encoding == enc::BGR8)
      return CV_8UC3;
    if (encoding == enc::MONO8)
      return CV_8UC1;
    if (encoding == enc::RGB8)
      return CV_8UC3;
    if (encoding == enc::MONO16)
      return CV_16UC1;
    if (encoding == enc::BGRA8)
      return CV_8UC4;
    if (encoding == enc::RGBA8)
      return CV_8UC4;

    // For bayer, return one-channel
    if (encoding == enc::BAYER_RGGB8)
      return CV_8UC1;
    if (encoding == enc::BAYER_BGGR8)
      return CV_8UC1;
    if (encoding == enc::BAYER_GBRG8)
      return CV_8UC1;
    if (encoding == enc::BAYER_GRBG8)
      return CV_8UC1;

    // Check all the generic content encodings
#define CHECK_ENCODING(code)                            \
    if (encoding == enc::TYPE_##code) return CV_##code    \
    /***/
#define CHECK_CHANNEL_TYPE(t)                   \
    CHECK_ENCODING(t##1);                         \
    CHECK_ENCODING(t##2);                         \
    CHECK_ENCODING(t##3);                         \
    CHECK_ENCODING(t##4);                         \
    /***/

    CHECK_CHANNEL_TYPE(8UC);
    CHECK_CHANNEL_TYPE(8SC);
    CHECK_CHANNEL_TYPE(16UC);
    CHECK_CHANNEL_TYPE(16SC);
    CHECK_CHANNEL_TYPE(32SC);
    CHECK_CHANNEL_TYPE(32FC);
    CHECK_CHANNEL_TYPE(64FC);

#undef CHECK_CHANNEL_TYPE
#undef CHECK_ENCODING

    throw std::runtime_error("Unrecognized image encoding [" + encoding + "]");
  }

  std::string
  mattype2enconding(int mat_type)
  {
    switch (mat_type)
    {
      case CV_8UC1:
        return enc::MONO8;
      case CV_8UC3:
        return enc::RGB8;
      case CV_16SC1:
        return enc::MONO16;
      case CV_8UC4:
        return enc::RGBA8;
      default:
        break;
    }

#define CASE_ENCODING(code)                    \
    case CV_##code : return enc::TYPE_##code; do{}while(false)    \
    /***/
#define CASE_CHANNEL_TYPE(t)                   \
    CASE_ENCODING(t##1);                         \
    CASE_ENCODING(t##2);                         \
    CASE_ENCODING(t##3);                         \
    CASE_ENCODING(t##4);                         \
    /***/
    switch (mat_type)
    {
      CASE_CHANNEL_TYPE(8UC);
      CASE_CHANNEL_TYPE(8SC);
      CASE_CHANNEL_TYPE(16UC);
      CASE_CHANNEL_TYPE(16SC);
      CASE_CHANNEL_TYPE(32SC);
      CASE_CHANNEL_TYPE(32FC);
      CASE_CHANNEL_TYPE(64FC);
      default:
        throw std::runtime_error("Unknown encoding type.");
    }

  }
  void
  toImageMsg(const cv::Mat& mat, sensor_msgs::Image& ros_image)
  {
    ros_image.height = mat.rows;
    ros_image.width = mat.cols;
    ros_image.encoding = mattype2enconding(mat.type());
    ros_image.is_bigendian = false;
    ros_image.step = mat.step;
    size_t size = mat.step * mat.rows;
    ros_image.data.resize(size);
    memcpy((char*) (&ros_image.data[0]), mat.data, size);
  }

  void
  toDepthImageMsg(const sensor_msgs::PointCloud& msg, sensor_msgs::Image& ros_image)
  {
    ros_image.height = msg.points.size();
    ros_image.width = 1;
    ros_image.encoding = mattype2enconding(CV_32F);
    ros_image.is_bigendian = false;
    ros_image.step = sizeof(float);
    size_t size = ros_image.step * ros_image.height;
    ros_image.data.resize(size);
    memcpy(reinterpret_cast<void*>(ros_image.data.data()), reinterpret_cast<const void*>(msg.points.data()), size);
  }

  void
  toDepthImageMsg(const sensor_msgs::PointCloud2& msg, sensor_msgs::Image& ros_image)
  {
    ros_image.height = msg.height;
    ros_image.width = msg.width;
    ros_image.encoding = mattype2enconding(CV_32F);
    ros_image.is_bigendian = false;
    ros_image.step = sizeof(float) * ros_image.width;
    size_t size = ros_image.step * ros_image.height;
    ros_image.data.resize(size);
    memcpy(reinterpret_cast<void*>(ros_image.data.data()), reinterpret_cast<const void*>(msg.data.data()), size);
  }

  void
  toPointCloud(const cv::Mat& cloud, sensor_msgs::PointCloud& msg)
  {
    msg.points.resize(cloud.rows);
    std::vector<geometry_msgs::Point32>::iterator out_it = msg.points.begin();
    const float* pi = cloud.ptr<float>(0);
    for (int i = 0; i < cloud.rows; i++)
    {
      geometry_msgs::Point32& p = *(out_it++);
      p.x = *(pi++);
      p.y = *(pi++);
      p.z = *(pi++);
    };
  }

  void
  toPointCloud(const cv::Mat& cloud, sensor_msgs::PointCloud2& msg)
  {
    cv::Mat wc;
    if (!cloud.isContinuous() || cloud.depth() != CV_32F)
    {
      cloud.copyTo(wc, CV_32F);
    }
    else
    {
      wc = cloud;
    }
    msg.data.resize(wc.total() * wc.elemSize());
    std::memcpy(msg.data.data(), wc.data, msg.data.size());
    msg.width = wc.rows;
    msg.height = 1;
    msg.fields.clear();
    sensor_msgs::PointField f;
    f.count = 1;
    f.datatype = sensor_msgs::PointField::FLOAT32;

    f.offset = 0;
    f.name = "x";

    msg.fields.push_back(f);

    f.offset += wc.elemSize();
    f.name = "y";

    msg.fields.push_back(f);

    f.offset += wc.elemSize();
    f.name = "z";
    msg.fields.push_back(f);

    msg.point_step = 3 * wc.elemSize();
    msg.row_step = msg.point_step * msg.width;
  }

}
namespace ecto_ros
{

  using ecto::tendrils;
  using std::string;
  using namespace sensor_msgs;

  struct Image2Mat
  {
    static void
    declare_params(ecto::tendrils& p)
    {
      p.declare<bool>("swap_rgb", "Swap the red and blue channels", false);
    }
    static void
    declare_io(const tendrils& /*p*/, tendrils& i, tendrils& o)
    {
      i.declare<ImageConstPtr>("image", "A sensor_msg::Image message from ros.");

      o.declare<cv::Mat>("image", "A cv::Mat copy.");
    }
    void
    configure(const tendrils& p, const tendrils& i, const tendrils& o)
    {
      image_msg_ = i["image"];
      mat_ = o["image"];
      swap_rgb_ = p.get<bool>("swap_rgb");
    }
    int
    process(const tendrils& i, const tendrils& o)
    {
      ImageConstPtr image = *image_msg_;
      cv::Mat mat;
      // Construct matrix pointing to source data
      int source_type = getCvType(image->encoding);
      cv::Mat temp((int) image->height, (int) image->width, source_type, const_cast<uint8_t*>(&image->data[0]),
                   (size_t) image->step);
      if (swap_rgb_)
        cv::cvtColor(temp, mat, CV_BGR2RGB);
      else
        temp.copyTo(mat);
      mat.copyTo(*mat_);
      return ecto::OK;
    }
    ecto::spore<ImageConstPtr> image_msg_;
    ecto::spore<cv::Mat> mat_;
    bool swap_rgb_;
  };

  struct Mat2Image
  {
    static void
    declare_params(tendrils& p)
    {
      p.declare<std::string>("frame_id", "Frame this data is associated with", "default_frame");
      p.declare<std::string>("encoding", "ROS image message encoding override.");
      p.declare<bool>("swap_rgb", "Swap the red and blue channels", false);

    }

    static void
    declare_io(const tendrils& /*p*/, tendrils& i, tendrils& o)
    {
      i.declare<cv::Mat>("image", "A cv::Mat.");
      o.declare<ImageConstPtr>("image", "A sensor_msg::Image message.");
    }

    void
    configure(const tendrils& p, const tendrils& i, const tendrils& o)
    {
      mat_ = i["image"];
      cloud_msg_out_ = o["image"];
      frame_id_ = p.get<std::string>("frame_id");
      header_.frame_id = frame_id_;
      encoding_ = p["encoding"];
      swap_rgb_ = p.get<bool>("swap_rgb");

    }
    int
    process(const tendrils& i, const tendrils& o)
    {
      ImagePtr image_msg(new Image);
      cv::Mat mat;
      if (swap_rgb_)
        cv::cvtColor(*mat_, mat, CV_BGR2RGB);
      else
        mat = *mat_;

      toImageMsg(mat, *image_msg);
      if (encoding_.user_supplied())
      {
        image_msg->encoding = *encoding_;
      }

      header_stanza(header_);
      image_msg->header = header_;
      *cloud_msg_out_ = image_msg;
      return ecto::OK;
    }
    std_msgs::Header header_;
    std::string frame_id_;
    ecto::spore<ImageConstPtr> cloud_msg_out_;
    ecto::spore<cv::Mat> mat_;
    ecto::spore<std::string> encoding_;
    bool swap_rgb_;
  };

  template<typename PointCloudT>
  struct Mat2PointCloud_
  {
    typedef typename PointCloudT::Ptr CloudPtr;
    typedef typename PointCloudT::ConstPtr CloudConstPtr;
    static void
    declare_params(tendrils& p)
    {
      p.declare<std::string>("frame_id", "Frame this data is associated with", "default_frame");
    }

    static void
    declare_io(const tendrils& /*p*/, tendrils& i, tendrils& o)
    {
      i.declare<cv::Mat>("image", "A cv::Mat.");
      o.declare<CloudConstPtr>("cloud", "A sensor_msg::PointCloud2 message.");
    }

    void
    configure(const tendrils& p, const tendrils& i, const tendrils& o)
    {
      mat_ = i["image"];
      cloud_msg_out_ = o["cloud"];
      frame_id_ = p.get<std::string>("frame_id");
      header_.frame_id = frame_id_;
    }

    int
    process(const tendrils& i, const tendrils& o)
    {
      CloudPtr cloud_msg(new PointCloudT());
      toPointCloud(*mat_, *cloud_msg);
      header_stanza(header_);
      cloud_msg->header = header_;
      *cloud_msg_out_ = cloud_msg;
      return ecto::OK;
    }
    std_msgs::Header header_;
    std::string frame_id_;
    ecto::spore<CloudConstPtr> cloud_msg_out_;
    ecto::spore<cv::Mat> mat_;
    ecto::spore<std::string> encoding_;
  };

  struct Mat2PointCloud: Mat2PointCloud_<sensor_msgs::PointCloud>
  {
  };

  struct Mat2PointCloud2: Mat2PointCloud_<sensor_msgs::PointCloud2>
  {
  };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  template<typename PointCloudT>
  struct PointCloud2DepthImage_
  {
    typedef typename PointCloudT::Ptr CloudPtr;
    typedef typename PointCloudT::ConstPtr CloudConstPtr;

    static void
    declare_io(const tendrils& /*p*/, tendrils& i, tendrils& o)
    {
      i.declare<CloudConstPtr>("cloud", "A sensor_msg::PointCloud2 message.");
      o.declare<ImageConstPtr>("image", "A cv::Mat with only one channel for the depth.");
    }

    void
    configure(const tendrils& p, const tendrils& i, const tendrils& o)
    {
      cloud_msg_ = i["cloud"];
      depth_msg_ = o["image"];
    }

    int
    process(const tendrils& i, const tendrils& o)
    {
      ImagePtr depth_msg(new Image());
      toDepthImageMsg(**cloud_msg_, *depth_msg);
      *depth_msg_ = depth_msg;
      return ecto::OK;
    }
    std_msgs::Header header_;
    std::string frame_id_;
    ecto::spore<CloudConstPtr> cloud_msg_;
    ecto::spore<ImageConstPtr> depth_msg_;
    ecto::spore<std::string> encoding_;
  };

  struct PointCloud2DepthImage: PointCloud2DepthImage_<sensor_msgs::PointCloud>
  {
  };

  struct PointCloud22DepthImage: PointCloud2DepthImage_<sensor_msgs::PointCloud2>
  {
  };
}

ECTO_CELL(ecto_ros_main, ecto_ros::Image2Mat, "Image2Mat", "Converts an Image message to cv::Mat type.")
ECTO_CELL(ecto_ros_main, ecto_ros::Mat2Image, "Mat2Image", "Converts an cv::Mat to Image message type.")
ECTO_CELL(ecto_ros_main, ecto_ros::Mat2PointCloud, "Mat2PointCloud", "Converts an cv::Mat to PointCloud.")
ECTO_CELL(ecto_ros_main, ecto_ros::Mat2PointCloud2, "Mat2PointCloud2", "Converts an cv::Mat to PointCloud2.")
ECTO_CELL(ecto_ros_main, ecto_ros::PointCloud2DepthImage, "PointCloud2DepthImage",
          "Converts a PointCloud to a depth Image message type.")
ECTO_CELL(ecto_ros_main, ecto_ros::PointCloud22DepthImage, "PointCloud22DepthImage",
          "Converts a PointCloud2 to a depth Image message type.")

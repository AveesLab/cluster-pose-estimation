/// Copyright (c) 2014,
/// Systems, Robotics and Vision Group
/// University of the Balearic Islands
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///     * Redistributions of source code must retain the above copyright
///       notice, this list of conditions and the following disclaimer.
///     * Redistributions in binary form must reproduce the above copyright
///       notice, this list of conditions and the following disclaimer in the
///       documentation and/or other materials provided with the distribution.
///     * All advertising materials mentioning features or use of this software
///       must display the following acknowledgement:
///       This product includes software developed by
///       Systems, Robotics and Vision Group, Univ. of the Balearic Islands
///     * Neither the name of Systems, Robotics and Vision Group, University of
///       the Balearic Islands nor the names of its contributors may be used
///       to endorse or promote products derived from this software without
///       specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
/// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
/// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
/// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
/// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
/// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
/// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef MONO_CAMERA_H
#define MONO_CAMERA_H

#include "avt_vimba_camera/avt_vimba_camera.hpp"
#include "avt_vimba_camera/avt_vimba_api.hpp"

// Basic Libraries
#include <string>
#include <thread>
#include <cmath>
#include <chrono>

// ROS
#include <avt_vimba_camera_msgs/srv/detail/load_settings__struct.hpp>
#include <avt_vimba_camera_msgs/srv/detail/save_settings__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <avt_vimba_camera_msgs/srv/load_settings.hpp>
#include <avt_vimba_camera_msgs/srv/save_settings.hpp>
#include <posenet_msgs/msg/keypoints.hpp>
#include <posenet_msgs/msg/links.hpp>
#include <posenet_msgs/msg/objectpose.hpp>
#include <posenet_msgs/msg/poses.hpp>

// Image Selection
#include "imageselection/imageselection.hpp"

// DNN Inference
#include "poseestimation/node_posenet.hpp"

// benchmark
#include <string>
#include <ctime>
#include <fstream>


namespace avt_vimba_camera
{
class MonoCameraNode : public rclcpp::Node
{
public:
  MonoCameraNode();
  ~MonoCameraNode();

  void Start();

private:
  void LoadParams();
  void FrameCallback(const FramePtr& vimba_frame_ptr);
  void ImageSelectionSynchronize(std_msgs::msg::Header::SharedPtr base_timestamp);

  // GigE Camera
  AvtVimbaApi api_;
  AvtVimbaCamera cam_;

  std::string ip_;
  std::string guid_;
  std::string camera_info_url_;
  std::string frame_id_;
  int32_t ptp_offset_;

  // Cluster System Information
  int node_index_;

  // Image Selection
  std::shared_ptr<ImageSelection> image_selection_;
  int number_of_nodes_;
  double camera_fps_;
  double local_inference_fps_;
  double timestamp_margin_milisecond_;

  // Image Selection : Synchronize
  int convert_frame_;
  int synchronization_cnt_;
  bool cluster_flag_;
  bool emergency_flag_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr cluster_synchronize_publisher_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr cluster_synchronize_subscriber_;

  // Publisher : Result
  rclcpp::Publisher<posenet_msgs::msg::Poses>::SharedPtr poses_publisher_;

  // Pose Estimation
  std::shared_ptr<PoseNet> inference_;

  // Benchmark
  std::fstream file_;
  long long int bench_startpoint;
  long long int bench_timestamp;
  long long int bench_aftergetimage;
  long long int bench_ismyframe;
  long long int bench_aftercluster;
  long long int bench_afterpreprocess;
  long long int bench_afterinference;
  long long int bench_afterpostprocess;
  long long int bench_numberofobject;
  long long int bench_afterpublish;
  long long int bench_endpoint;
};
}  // namespace avt_vimba_camera
#endif

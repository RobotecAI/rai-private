// Copyright (C) 2024 Robotec.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//         http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#if defined(ROS_DISTRO_HUMBLE)
#include <cv_bridge/cv_bridge.h>
#elif defined(ROS_DISTRO_JAZZY)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.hpp>  // Default to .hpp for future distributions
#endif

#include <mutex>
#include <opencv2/opencv.hpp>
#include <opencv2/quality/qualityssim.hpp>
#include <rai_interfaces/srv/what_i_see.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

class WhatISeeNode : public rclcpp::Node
{
public:
  WhatISeeNode() : Node("rai_whatisee_node")
  {
    constexpr float observation_interval_seconds_default = 0.5;  // Look only every X seconds
    constexpr float image_similarity_threshold_default = 0.9f;   // Below threshold are considered
    const std::string camera_color_topic_default = "/camera/color/image_raw";
    this->declare_parameter("camera_color_topic", camera_color_topic_default);
    this->declare_parameter("observation_interval_seconds", observation_interval_seconds_default);
    this->declare_parameter("image_similarity_threshold", image_similarity_threshold_default);

    camera_color_topic_ = get_parameter("camera_color_topic").as_string();
    try {
      rclcpp::expand_topic_or_service_name(camera_color_topic_, get_name(), get_namespace());
    } catch (const rclcpp::exceptions::InvalidTopicNameError & e) {
      std::cerr << "Invalid topic name: " << e.what() << std::endl;
    }

    image_similarity_threshold_ = get_parameter("image_similarity_threshold").as_double();
    if (image_similarity_threshold_ > 1.0f || image_similarity_threshold_ < 0.0f) {
      RCLCPP_WARN(get_logger(), "Image similarity threshold should be between 1 and 0");
      image_similarity_threshold_ = std::clamp<float>(image_similarity_threshold_, 0.0f, 1.0f);
    }

    auto interval_seconds = get_parameter("observation_interval_seconds").as_double();
    if (interval_seconds < 0.0) {
      RCLCPP_WARN(get_logger(), "Invalid observation interval parameter, setting to 0");
      interval_seconds = 0.0;
    }
    observation_interval_ = rclcpp::Duration::from_seconds(interval_seconds);

    camera_color_image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      camera_color_topic_, rclcpp::SensorDataQoS(),
      std::bind(&WhatISeeNode::image_callback, this, std::placeholders::_1));

    anything_new_srv_ = create_service<std_srvs::srv::Trigger>(
      "rai/whatisee/anything_new",
      std::bind(&WhatISeeNode::anything_new, this, std::placeholders::_1, std::placeholders::_2));

    whatisee_srv_ = create_service<rai_interfaces::srv::WhatISee>(
      "rai/whatisee/get",
      std::bind(&WhatISeeNode::what_i_see, this, std::placeholders::_1, std::placeholders::_2));

    last_observation_timestamp_ = get_clock()->now();
  }

private:
  void anything_new(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!fresh_color_camera_image_) {
      response->success = false;
      return;
    }

    if (!last_color_camera_image_) {
      response->success = true;
      return;
    }

    response->success =
      fresh_color_camera_image_->header.stamp != last_color_camera_image_->header.stamp;
  }

  void what_i_see(
    [[maybe_unused]] const std::shared_ptr<rai_interfaces::srv::WhatISee::Request> request,
    const std::shared_ptr<rai_interfaces::srv::WhatISee::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!fresh_color_camera_image_) {
      response->observations.push_back("no image");
      return;
    }

    response->observations.push_back("nothing to add");
    response->perception_source = camera_color_topic_;
    response->image = *fresh_color_camera_image_;
    last_color_camera_image_ = fresh_color_camera_image_;
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    auto time_now = get_clock()->now();
    if (time_now - last_observation_timestamp_ > observation_interval_) {
      if (is_image_novel(msg)) {
        std::lock_guard<std::mutex> lock(mutex_);
        fresh_color_camera_image_ = msg;
      }
      last_observation_timestamp_ = time_now;
    }
  }

  // we compare to the last requested message, not to the recent message
  bool is_image_novel([[maybe_unused]] const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!last_color_camera_image_) {
      return true;
    }
    cv::Mat new_img_opencv;
    cv::Mat old_img_opencv;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      try {
        new_img_opencv = cv_bridge::toCvCopy(msg, "mono8")->image;
        // TODO - save the old conversion
        old_img_opencv = cv_bridge::toCvCopy(last_color_camera_image_, "mono8")->image;
      } catch (cv_bridge::Exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to convert image: %s", e.what());
        return false;
      }
    }
    const cv::Scalar ssim =
      cv::quality::QualitySSIM::compute(new_img_opencv, old_img_opencv, cv::noArray());
    return ssim[0] < image_similarity_threshold_;
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_color_image_subscription_;
  rclcpp::Service<rai_interfaces::srv::WhatISee>::SharedPtr whatisee_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr anything_new_srv_;

  rclcpp::Duration observation_interval_{0, 0};  // Zero is allowed, means no interval filtering
  float image_similarity_threshold_;
  std::string camera_color_topic_;
  rclcpp::Time last_observation_timestamp_;
  sensor_msgs::msg::Image::SharedPtr fresh_color_camera_image_;
  sensor_msgs::msg::Image::SharedPtr last_color_camera_image_;
  std::mutex mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WhatISeeNode>());
  rclcpp::shutdown();
  return 0;
}

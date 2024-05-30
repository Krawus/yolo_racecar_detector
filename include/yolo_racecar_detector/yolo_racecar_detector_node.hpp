// Copyright 2024 akrawczyk
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef YOLO_RACECAR_DETECTOR__YOLO_RACECAR_DETECTOR_NODE_HPP_
#define YOLO_RACECAR_DETECTOR__YOLO_RACECAR_DETECTOR_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include "object_recognition_utils/object_classification.hpp"


#include "yolo_racecar_detector/yolo_racecar_detector.hpp"

namespace yolo_racecar_detector
{
using YoloRacecarDetectorPtr = std::unique_ptr<yolo_racecar_detector::YoloRacecarDetector>;

class YOLO_RACECAR_DETECTOR_PUBLIC YoloRacecarDetectorNode : public rclcpp::Node
{
public:
  explicit YoloRacecarDetectorNode(const rclcpp::NodeOptions & options);

private:
  YoloRacecarDetectorPtr yolo_racecar_detector_{nullptr};
  int64_t param_name_{123};

  void cameraImgCallback(const sensor_msgs::msg::Image::SharedPtr imgMsg);

  tier4_perception_msgs::msg::DetectedObjectsWithFeature bboxesToDetectedObjectsWithFeatureMsg(const std::vector<BBoxInfo> & bboxes);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_img_sub_;
  rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr detected_cars_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_with_bboxes_pub_;


};
}  // namespace yolo_racecar_detector

#endif  // YOLO_RACECAR_DETECTOR__YOLO_RACECAR_DETECTOR_NODE_HPP_

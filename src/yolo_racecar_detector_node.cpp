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

#include "yolo_racecar_detector/yolo_racecar_detector_node.hpp"

namespace yolo_racecar_detector
{

YoloRacecarDetectorNode::YoloRacecarDetectorNode(const rclcpp::NodeOptions & options)
:  Node("yolo_racecar_detector", options)
{
  // get paths params
  float nms_threshold = this->declare_parameter("nms_threshold", 0.4);
  float confidence_threshold = this->declare_parameter("confidendce_threshold", 0.4);
  std::string onnx_path = this->declare_parameter("onnx_model_path", "models/model.onnx");
  std::string engine_path = this->declare_parameter("trt_engine_path", "engines/engine.trt");

  yolo_racecar_detector_ = std::make_unique<yolo_racecar_detector::YoloRacecarDetector>(onnx_path, engine_path, confidence_threshold, nms_threshold);
  
  auto camera_img_sub_qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

  camera_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/in/image", camera_img_sub_qos, 
                                              std::bind(&YoloRacecarDetectorNode::cameraImgCallback, this, std::placeholders::_1));

  detected_cars_pub_ = this->create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>("out/objects", 10);
  image_with_bboxes_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/out/image", 10);


}

 tier4_perception_msgs::msg::DetectedObjectsWithFeature YoloRacecarDetectorNode::bboxesToDetectedObjectsWithFeatureMsg(const std::vector<BBoxInfo> & bboxes){
  
  tier4_perception_msgs::msg::DetectedObjectsWithFeature objects_with_feature_msg;

  for (auto& bbox : bboxes){
    tier4_perception_msgs::msg::DetectedObjectWithFeature object_with_feature;
    object_with_feature.feature.roi.width = (bbox.box.x2 - bbox.box.x1);
    object_with_feature.feature.roi.height = (bbox.box.y2 - bbox.box.y1);
    object_with_feature.feature.roi.x_offset = bbox.box.x1;
    object_with_feature.feature.roi.y_offset = bbox.box.y1;
    object_with_feature.object.existence_probability = bbox.prob;

    std::cout << "DETECTED RACECAR WITH PROB: " << bbox.prob << std::endl;
    objects_with_feature_msg.feature_objects.push_back(object_with_feature);
  }

  return objects_with_feature_msg;

}


void YoloRacecarDetectorNode::cameraImgCallback(const sensor_msgs::msg::Image::SharedPtr imgMsg)
{
  cv::Mat img = cv_bridge::toCvCopy(imgMsg, "bgr8")->image;
  std::vector<BBoxInfo> bboxes = yolo_racecar_detector_->runInference(img);
  std::vector<BBoxInfo> carBboxes = yolo_racecar_detector_->getFilteredBboxes(bboxes);

  tier4_perception_msgs::msg::DetectedObjectsWithFeature objects_with_feature_msg = bboxesToDetectedObjectsWithFeatureMsg(carBboxes);

  cv::Mat imageWithBboxes = yolo_racecar_detector_->getImageWithBboxes(img, carBboxes);
  std::shared_ptr<sensor_msgs::msg::Image> image_with_bboxes_msg_ptr = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", imageWithBboxes).toImageMsg();
  sensor_msgs::msg::Image image_with_bboxes_msg = *image_with_bboxes_msg_ptr;

  detected_cars_pub_->publish(objects_with_feature_msg);
  image_with_bboxes_pub_->publish(image_with_bboxes_msg);

}


}  // namespace yolo_racecar_detector

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(yolo_racecar_detector::YoloRacecarDetectorNode)

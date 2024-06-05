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

#ifndef YOLO_RACECAR_DETECTOR__YOLO_RACECAR_DETECTOR_HPP_
#define YOLO_RACECAR_DETECTOR__YOLO_RACECAR_DETECTOR_HPP_

#include <cstdint>
#include <filesystem>
#include <string>
#include <opencv2/opencv.hpp>

#include "yolo_racecar_detector/visibility_control.hpp"
#include "yolo_racecar_detector/engine.hpp"


namespace yolo_racecar_detector
{

class YOLO_RACECAR_DETECTOR_PUBLIC YoloRacecarDetector
{
public:
  static const int CAR_LABEL_ID = 0;

  YoloRacecarDetector(const std::string& onnx_path, const std::string& engine_path, const float& confidence_threshold, const float& nms_threshold_);
  bool engineExists(const std::string& engine_path);

  std::vector<BBoxInfo> runInference(cv::Mat& img);
  std::vector<BBoxInfo> getFilteredBboxes(const std::vector<BBoxInfo>& bboxes);
  cv::Mat getImageWithBboxes(const cv::Mat& img, const std::vector<BBoxInfo>& bboxes);

private:
  float confidence_threshold_ = 0.4;
  float nms_threshold_ = 0.4;
  Engine trt_engine_;
  
};

}  // namespace yolo_racecar_detector

#endif  // YOLO_RACECAR_DETECTOR__YOLO_RACECAR_DETECTOR_HPP_

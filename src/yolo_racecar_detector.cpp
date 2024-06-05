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

#include "yolo_racecar_detector/yolo_racecar_detector.hpp"

#include <iostream>


#define NMS_THRESH 0.4
#define CONF_THRESH 0.3




namespace yolo_racecar_detector
{

cv::Rect get_rect(BBox box) {
  return cv::Rect(round(box.x1), round(box.y1), round(box.x2 - box.x1), round(box.y2 - box.y1));
  }

YoloRacecarDetector::YoloRacecarDetector(const std::string& onnx_path, const std::string& engine_path, const float& confidence_threshold, const float& nms_threshold){

  confidence_threshold_ = confidence_threshold;
  nms_threshold_ = nms_threshold;
  std::cout << "model path: " << onnx_path << std::endl;
  
  if (!engineExists(engine_path)){
    std::cout << "ENGINE FILE NOT FOUND. GENERATING..." << std::endl;
    OptimDim dyn_dim_profile;
    dyn_dim_profile.tensor_name = "images";
    dyn_dim_profile.size = nvinfer1::Dims4{1, 3, 480, 640};

    trt_engine_.build_engine(onnx_path, engine_path, dyn_dim_profile);

    if (!engineExists(engine_path)){
      std::cerr << "FAILED TO GENERATE ENGINE" << std::endl;
      std::cerr << "Shutting down..." << std::endl;
      exit(1);
    }

    std::cout << "TRT engine build finished" << std::endl;
    
  }

  trt_engine_.init(engine_path);

  if(trt_engine_.isInit())
    std::cout << "TRT engine initialized" << std::endl;
  else{
    std::cerr << "TRT engine not initialized" << std::endl;
    std::cerr << "Shutting down..." << std::endl;
    exit(1);
  }

}


bool YoloRacecarDetector::engineExists(const std::string& enginePath)
{
  std::fstream f(enginePath.c_str());
  return f.good();
}

std::vector<BBoxInfo> YoloRacecarDetector::runInference(cv::Mat& img)
{
  cv::Mat resized_img;
  cv::resize(img, resized_img, cv::Size(img.cols / 2, img.rows / 2));

  return trt_engine_.run(img, img.rows, img.cols, confidence_threshold_, nms_threshold_);
}

std::vector<BBoxInfo> YoloRacecarDetector::getFilteredBboxes(const std::vector<BBoxInfo>& bboxes)
{
  std::vector<BBoxInfo> carBboxes;
  for (size_t j = 0; j < bboxes.size(); j++) {
    if (bboxes[j].label == CAR_LABEL_ID){
      carBboxes.push_back(bboxes[j]);
    }
  }
  return carBboxes;
}

cv::Mat YoloRacecarDetector::getImageWithBboxes(const cv::Mat& img, const std::vector<BBoxInfo>& bboxes)
{
  cv::Mat img_with_bboxes = img.clone();
  for (size_t j = 0; j < bboxes.size(); j++) {
      cv::Rect r = get_rect(bboxes[j].box);
      cv::rectangle(img_with_bboxes, r, cv::Scalar(0, 165, 255), 4);
  }
  return img_with_bboxes;
}





}  // namespace yolo_racecar_detector

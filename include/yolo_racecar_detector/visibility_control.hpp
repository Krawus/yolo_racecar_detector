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

#ifndef YOLO_RACECAR_DETECTOR__VISIBILITY_CONTROL_HPP_
#define YOLO_RACECAR_DETECTOR__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(YOLO_RACECAR_DETECTOR_BUILDING_DLL) || defined(YOLO_RACECAR_DETECTOR_EXPORTS)
    #define YOLO_RACECAR_DETECTOR_PUBLIC __declspec(dllexport)
    #define YOLO_RACECAR_DETECTOR_LOCAL
  #else  // defined(YOLO_RACECAR_DETECTOR_BUILDING_DLL) || defined(YOLO_RACECAR_DETECTOR_EXPORTS)
    #define YOLO_RACECAR_DETECTOR_PUBLIC __declspec(dllimport)
    #define YOLO_RACECAR_DETECTOR_LOCAL
  #endif  // defined(YOLO_RACECAR_DETECTOR_BUILDING_DLL) || defined(YOLO_RACECAR_DETECTOR_EXPORTS)
#elif defined(__linux__)
  #define YOLO_RACECAR_DETECTOR_PUBLIC __attribute__((visibility("default")))
  #define YOLO_RACECAR_DETECTOR_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define YOLO_RACECAR_DETECTOR_PUBLIC __attribute__((visibility("default")))
  #define YOLO_RACECAR_DETECTOR_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // YOLO_RACECAR_DETECTOR__VISIBILITY_CONTROL_HPP_

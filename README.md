# yolo_racecar_detector
<!-- Required -->
<!-- Package description -->

## Installation
<!-- Required -->
<!-- Things to consider:
    - How to build package? 
    - Are there any other 3rd party dependencies required? -->

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=On --packages-up-to yolo_racecar_detector
```

## Usage
<!-- Required -->
<!-- Things to consider:
    - Launching package. 
    - Exposed API (example service/action call. -->

```bash
ros2 launch yolo_racecar_detector yolo_racecar_detector.launch.py
```

## API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

### Input

| Name       | Type                | Description     |
| ---------- | ------------------- | --------------- |
| `in/image` | `sensor_msgs/Image` | The input image |

### Output

| Name          | Type                                               | Description                                        |
| ------------- | -------------------------------------------------- | -------------------------------------------------- |
| `out/objects` | `tier4_perception_msgs/DetectedObjectsWithFeature` | The detected objects with 2D bounding boxes        |
| `out/image`   | `sensor_msgs/Image`                                | The image with 2D bounding boxes for visualization |


### Parameters

| Name         | Type | Description  |
| ------------ | ---- | ------------ |
| `onnx_model_path` | string  | relative path to yolo model in onnx     format |
| `trt_engine_path` | string | relative path to trt engine (will be generated if does not exist)|
|`confidendce_threshold` | float | If car detection confidence score is less than this value, the object is ignored.|
|`nms_threshold` | float | The IoU threshold for Non-maximum Suppression|



## References / External links
<!-- Optional -->

# Turtlebot3 Object Tracker

This project implements an object tracking system for a Turtlebot3 robot using the YOLO (You Only Look Once) algorithm. The system can detect and track objects of interest in real-time using a camera mounted on the robot. The project consists of two main components: `image_processor` and `controller`.

![ezgif-1-80f7319f72](https://github.com/PouriaAzadehR/Object-Trakcer-Robot/assets/93463377/eeda6ef1-fa9d-4701-8fed-3313e0fcf594)
## Prerequisites
- Ubuntu OS
- ROS (Robot Operating System) installed
- Python 3

## Installation

1. Clone the project repository:
   ```
   git clone <repository_url>
   ```

2. Install the required dependencies:
   ```
   pip install numpy cv2 rosnumpy ultralytics torch
   ```

3. Build the project:
   ```
   cd turtlebot3_object_tracker
   catkin_make
   ```

## Usage

### Image Processor Node

The `image_processor` node is responsible for processing the camera images and performing object detection using the YOLO algorithm. It subscribes to the robot's camera topic and publishes the processed images with bounding box annotations. To run the `image_processor` node, execute the following command:

```
rosrun turtlebot3_object_tracker image_processor.py
```

### Controller Node

The `controller` node is responsible for controlling the robot based on the detected objects. It uses a proportional controller to adjust the robot's angular velocity and follows the detected object. To run the `controller` node, execute the following command:

```
rosrun turtlebot3_object_tracker controller.py
```

## ROS Services

### `detection` Service

The `detection` service is used to request object detection information from the `image_processor` node. It takes a `label` string as input and returns a response message containing information about the detected object. The response message includes the following fields:

- `exists` (bool): Indicates whether an object with the specified label exists in the image.
- `pos_x_in_pic` (float64): X-coordinate of the object's center in the image.
- `pos_y_in_pic` (float64): Y-coordinate of the object's center in the image.
- `bb_width` (float64): Width of the bounding box around the object.
- `bb_height` (float64): Height of the bounding box around the object.
- `pic_width` (float64): Width of the image.
- `pic_height` (float64): Height of the image.

To request object detection information, you can call the `detection` service using the provided service client.

## Customization

You can customize the project according to your specific requirements. Some possible modifications include:

- Modifying the YOLO model: You can use a different pre-trained YOLO model or train your own model for object detection.
- Changing the subscribed camera topic: Update the `camera_subscriber` in the `ImageProcessor` class to subscribe to the desired camera topic.
- Adjusting the control parameters: You can modify the angular velocity coefficient (`angular_vel_coef`) and the tolerance value (`e_l`) in the `Controller` class to achieve the desired tracking behavior.

Feel free to explore and adapt the project to meet your specific needs.

## Contributing

Contributions to this project are welcome. You can contribute by submitting bug reports, feature requests, or pull requests on the project repository.


## Acknowledgements

- [Ultralytics YOLO](https://github.com/ultralytics/yolov5): The Ultralytics YOLO library is used for object detection and tracking.
- [ROS](https://www.ros.org/): The Robot Operating System provides the framework for communication and control in this project.

## Contact



For any inquiries or questions, please contact [pouriazadeh81@gmail.com](mailto:pouriazadeh81@gmail.com).


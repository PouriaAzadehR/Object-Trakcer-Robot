#!/usr/bin/python3

# Python
import copy

# Object detection
import cv2
import numpy as np
from ultralytics import YOLO
from ultralytics.yolo.utils.plotting import Annotator
from ultralytics.yolo.engine.results import Results

# ROS
import rospy
from sensor_msgs.msg import Image

#srv
from turtlebot3_object_tracker.srv import detection, detectionResponse


class ImageProcessor:
    def __init__(self) -> None:
        # Image message
        self.image_msg = Image()

        self.image_res = 240, 320, 3 # Camera resolution: height, width
        self.image_np = np.zeros(self.image_res) # The numpy array to pour the image data into

        # TODO: Subscribe on your robot's camera topic
        # NOTE: Make sure you use the provided listener for this subscription
        self.camera_subscriber =  rospy.Subscriber("/follower/camera/image",Image, callback=self.camera_listener)


        # TODO: Instantiate your YOLO object detector/classifier model
        self.model: YOLO = YOLO('yolov8n.pt')
        # TODO: You need to update results each time you call your model

        self.results: Results = self.model(self.image_np)

        self.cv2_frame_size = 400, 320
        cv2.namedWindow("robot_view", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("robot_view", *self.cv2_frame_size)

        # TODO: Setup your "human detection" service
        self.human_detection_server = rospy.Service('/detection', detection, self.detection_handler)

        self.update_view()


    def detection_handler(self,req):
        label = req.label
        self.results: Results = self.model(self.image_np)
        for result in self.results:
            boxes = result.boxes.cpu().numpy()                         # get boxes on cpu in numpy
            for box in boxes:                                          # iterate boxes
                cls = box.cls[0].astype(int)  
                if self.model.names[cls] == label:
                    response = detectionResponse()
                    response.exists = True
                    r = box.xywh[0].astype(int)   
                    response.pos_x_in_pic = r[0]   
                    response.pos_y_in_pic = r[1] 
                    response.bb_width = r[2] 
                    response.bb_height = r[3]  
                    response.pic_width = self.image_res[0]
                    response.pic_height = self.image_res[1]
                    return response

        response = detectionResponse()
        response.exists = False
        return response



    def camera_listener(self, msg: Image):
        self.image_msg.data = copy.deepcopy(msg.data)


    def update_view(self):
        try:
            while not rospy.is_shutdown():
                if len(self.image_msg.data) == 0: # If there is no image data
                    continue

                # Convert binary image data to numpy array
                self.image_np = np.frombuffer(self.image_msg.data, dtype=np.uint8)
                self.image_np = self.image_np.reshape(self.image_res)

                frame = copy.deepcopy(self.image_np)

                # TODO: You can use an "Annotator" to draw object bounding boxes on frame
                for r in self.results:
                    annotator = Annotator(frame)
                    boxes = r.boxes
                    for box in boxes:
                        b = box.xyxy[0]  # get box coordinates in (top, left, bottom, right) format
                        c = box.cls
                        annotator.box_label(b, self.model.names[int(c)])
          
                frame = annotator.result() 
                cv2.imshow("robot_view", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                cv2.waitKey(1)

        except rospy.exceptions.ROSInterruptException:
            pass


if __name__ == "__main__":
    rospy.init_node("image_processor", anonymous=True)

    rospy.on_shutdown(cv2.destroyAllWindows)

    image_processor = ImageProcessor()

    rospy.spin()



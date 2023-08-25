import cv2
import dlib
import rclpy
from rclpy.node import Node
from utils2 import *
from std_msgs.msg import String,Float32

class DMSVisionNode(Node):
    def __init__(self):
        super().__init__("dms_vision")      #super function initialies node, here arguement is the name of the node.

        # Initialize the ROS2 publishers for driver_status, eye_open_ratio, and head_status
        self.eye_mode_pub = self.create_publisher(String, 'eye_mode', 10)
        self.driver_status_pub = self.create_publisher(String,'driver_status',10)
        self.head_pose_pub = self.create_publisher(String,'head_pose',10)
        self.eye_aspect_ratio_pub = self.create_publisher(Float32,'eye_aspect_ratio',10)
        
        # Load the dlib face detector and facial landmark predictor
        self.face_detector = dlib.get_frontal_face_detector()
        self.shape_predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

        # Change according to the patient
        self.patient_blink_threshold = 3.7
        self.patient_drowsiness_threshold = 1.7

        # asleep conditions
        self.asleep_duration = 0
        self.is_asleep = False

        # Access the camera feed
        self.cap = cv2.VideoCapture(0)

        # Call the function to start processing video and publishing results
        self.process_video()
    
    def process_video(self):
        while not self.is_shutdown():
            ret, frame = self.cap.read()

            if not ret:
                break

            # Convert the frame to grayscale for better face detection
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect faces in the frame
            faces = self.face_detector(gray_frame)

            # Process each detected face
            for face in faces:
                
                x1, y1 = face.left(), face.top()
                x2, y2 = face.right(), face.bottom()
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 1)


                # Get the facial landmarks
                landmarks = self.shape_predictor(gray_frame, face)

                # Compute driver_status, eye_open_ratio, and head_status
                eye_aspect_ratio = get_eye_aspect_ratio(gray_frame,landmarks)  # Replace with your eye open ratio computation
                eye_aspect_ratio = round(eye_aspect_ratio,3)
                head_pose = get_head_pose(frame,landmarks)
                mouth_open_ratio = get_mouth_aspect_ratio(gray_frame,landmarks)


                ear_msg = Float32()
                eye_mode_msg = String()
                head_pose_msg = String()
                driver_status_msg = String()


                
                driver_status = "Attentive"

                # finding eye_mode i.e open or closed

                if eye_aspect_ratio > self.patient_blink_threshold:
                    eye_mode = "Closed"
                    self.asleep_duration +=1
                    if self.asleep_duration >= 2 * 15:  # Assuming the frame rate is 30 fps
                        self.is_asleep = True
                
                else:
                    eye_mode = "Open"
                    self.asleep_duration = 0
                    self.is_asleep = False
                
                if self.is_asleep:
                    driver_status = "Asleep"
                
                if head_pose != "Straight":
                    driver_status = "Distracted"
                
                
                eye_mode_msg.data = str(eye_mode)
                ear_msg.data = eye_aspect_ratio
                driver_status_msg.data = str(driver_status)
                head_pose_msg.data = str(head_pose)
                

                # Publish the computed values
               
                self.eye_mode_pub.publish(eye_mode_msg)
                self.driver_status_pub.publish(driver_status_msg)
                self.head_pose_pub.publish(head_pose_msg)
                self.eye_aspect_ratio_pub.publish(ear_msg)

            cv2.imshow("Feed",frame)
            cv2.waitKey(1)


    def is_shutdown(self):
        return False  # Implement a proper shutdown condition if needed


def main():
    rclpy.init()        # initialise the ROS DDS function, helps us use ros functionality

    my_pub = DMSVisionNode() # created an instance of the class in the main function

    print("Publisher node is Running .....")
    
    # Note, this will only run once. To keep running, we use the following command:

    try:
        rclpy.spin(my_pub)  # rclpy's spin function keeps it running
    except KeyboardInterrupt:
        my_pub.destroy_node()   # this destroys the node 
        rclpy.shutdown()        # this shutdowns the dds connection






if __name__ == '__main__':
    main()
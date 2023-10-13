import cv2
import dlib
import rclpy
from rclpy.node import Node
from utils2 import *
from std_msgs.msg import String,Float32
import matplotlib.pyplot as plt
plt.style.use('dark_background')

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
        self.patient_blink_threshold = 3.8
        self.patient_drowsiness_threshold = 1.7

        # asleep conditions
        self.asleep_duration = 0
        self.is_asleep = False

        self.blink = 0

        # Create lists to store time and eye_open_ratio values
        self.time_values = []
        self.eye_open_ratio_values = []
        self.right_eye_open_ratio_values = []

        # Create a figure and axis for the plot
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(8, 6))

        # Set initial x-axis limits
        self.max_time = 100  # Adjust the value based on your requirements
        plt.xlim(0, self.max_time)


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
                #cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 1)


                # Get the facial landmarks
                landmarks = self.shape_predictor(gray_frame, face)
                for i in range(68):  # Assuming 68 landmarks are available
                    x, y = landmarks.part(i).x, landmarks.part(i).y
                    cv2.circle(frame, (x, y), 2, (0, 0, 255), -1)

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
                    self.blink = 1
                    self.asleep_duration +=1
                    if self.asleep_duration >= 2 * 8:  # Assuming the frame rate is 30 fps
                        self.is_asleep = True
                
                else:
                    eye_mode = "Open"
                    self.blink = 0
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

                # Append current time and eye_open_ratio values
                self.time_values.append(len(self.time_values) + 1)
                self.eye_open_ratio_values.append(self.blink)
                self.right_eye_open_ratio_values.append(eye_aspect_ratio)


            # Plot eye_open_ratio versus time
            self.ax1.plot(self.time_values, self.eye_open_ratio_values, color='red')
            self.ax1.set_xlabel('Time')
            self.ax1.set_ylabel('Blink')

            # Plot right_eye_open_ratio versus time
            self.ax2.plot(self.time_values, self.right_eye_open_ratio_values, color='blue')
            self.ax2.set_xlabel('Time')
            self.ax2.set_ylabel('Eye Aspect Ratio')

            # Adjust x-axis limits based on the number of data points
            if len(self.time_values) > self.max_time:
                self.ax1.set_xlim(len(self.time_values) - self.max_time, len(self.time_values))
                self.ax2.set_xlim(len(self.time_values) - self.max_time, len(self.time_values))
            else:
                self.ax1.set_xlim(0, self.max_time)
                self.ax2.set_xlim(0, self.max_time)

            # Update the plots
            plt.tight_layout()
            plt.pause(0.01)
            self.ax1.cla()
            self.ax2.cla()
            
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
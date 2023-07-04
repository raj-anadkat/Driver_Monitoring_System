import cv2
import numpy as np
import dlib
import matplotlib.pyplot as plt
from facial_detector import *


detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

# Change according to the patient
patient_blink_threshold = 3.7
patient_drowsiness_threshold = 1.7

# asleep conditions
asleep_duration = 0
is_asleep = False

cap = cv2.VideoCapture(0)

# Create lists to store time and eye_open_ratio values
time_values = []
eye_open_ratio_values = []
right_eye_open_ratio_values = []

# Create a figure and axis for the plot
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))

# Set initial x-axis limits
max_time = 100  # Adjust the value based on your requirements
plt.xlim(0, max_time)

while True:
    ret, frame = cap.read()
    frame = cv2.resize(frame, (640, 480))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = detector(gray)

    for face in faces:
        x1, y1 = face.left(), face.top()
        x2, y2 = face.right(), face.bottom()
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 1)
        cv2.putText(frame, "Driver: Raj", (x1-20, y1-10), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.6, (255, 255, 255))

        landmarks = predictor(gray, face)
        iris_left, iris_right = detect_iris(frame, landmarks)
        re_eye_open_ratio, le_eye_open_ratio = get_eye_open_ratio(frame, landmarks)
        drowsiness_ratio = get_drowsiness_ratio(frame, landmarks)

        print(drowsiness_ratio)
        

        left_eye_open_ratio = le_eye_open_ratio
        right_eye_open_ratio = re_eye_open_ratio
        eye_open_ratio = (left_eye_open_ratio + right_eye_open_ratio) / 2

        cv2.putText(frame, "Eye Mode:", (30, 30), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.8, (0, 0, 0))
        cv2.putText(frame, "Status:", (30, 50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.8, (0, 0, 0))
        cv2.putText(frame, "Head Pose:", (30, 70), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.8, (0, 0, 0))
        cv2.putText(frame, "Eye AR:", (30, 90), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.8, (0, 0, 0))

        driver_status = "Attentive"
        head_status = get_head_pose(frame, landmarks)

        if eye_open_ratio > patient_blink_threshold:
            cv2.putText(frame, "Closed", (160, 30), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.8, (0, 0, 255))
            eye_open_ratio = 1
            asleep_duration += 1
            if asleep_duration >= 2 * 15:  # Assuming the frame rate is 30 fps
                is_asleep = True
        else:
            eye_open_ratio = 0
            cv2.putText(frame, "Open", (160, 30), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.8, (0, 0, 0))
            asleep_duration = 0
            is_asleep = False

        if drowsiness_ratio < patient_drowsiness_threshold:
            driver_status = "Drowsy"
            cv2.putText(frame, driver_status, (160, 50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.8, (0, 0, 255))
        elif is_asleep:
            driver_status = "Asleep"
            cv2.putText(frame, driver_status, (160, 50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.8, (0, 0, 255))
        elif head_status != "Straight":
            driver_status = "Distracted"
            cv2.putText(frame, driver_status, (160, 50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.8, (0, 0, 255))
        
        elif drowsiness_ratio > 1.7 and drowsiness_ratio < 2.:
            driver_status = "Talking"
            cv2.putText(frame, driver_status, (160, 50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.8, (0, 0, 255))
        else:
            driver_status = "Attentive"
            cv2.putText(frame, driver_status, (160, 50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.8, (0, 0, 0))

        if head_status == "Straight":
            cv2.putText(frame, head_status, (160, 70), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.8, (0, 0, 0))
        else:
            cv2.putText(frame, head_status, (160, 70), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.8, (0, 0, 255))

        cv2.putText(frame, str(round(le_eye_open_ratio, 2)), (169, 90), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.8, (0, 0, 0))

        # Append current time and eye_open_ratio values
        time_values.append(len(time_values) + 1)
        eye_open_ratio_values.append(eye_open_ratio)
        right_eye_open_ratio_values.append(right_eye_open_ratio)

    # Plot eye_open_ratio versus time
    ax1.plot(time_values, eye_open_ratio_values, color='red')
    ax1.set_xlabel('Time')
    ax1.set_ylabel('Blink')

    # Plot right_eye_open_ratio versus time
    ax2.plot(time_values, right_eye_open_ratio_values, color='blue')
    ax2.set_xlabel('Time')
    ax2.set_ylabel('Eye Aspect Ratio')

    # Adjust x-axis limits based on the number of data points
    if len(time_values) > max_time:
        ax1.set_xlim(len(time_values) - max_time, len(time_values))
        ax2.set_xlim(len(time_values) - max_time, len(time_values))
    else:
        ax1.set_xlim(0, max_time)
        ax2.set_xlim(0, max_time)

    # Update the plots
    plt.tight_layout()
    plt.pause(0.01)
    ax1.cla()
    ax2.cla()

    cv2.imshow("Feed", frame)

    key = cv2.waitKey(1)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()

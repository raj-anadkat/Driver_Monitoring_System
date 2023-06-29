import cv2
import numpy as np
import dlib
import matplotlib.pyplot as plt

cap = cv2.VideoCapture(0)

detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

# Change according to the patient
patient_blink_threshold = 3.7

def midPoint(p1,p2):
    return (int((p1.x +p2.x)*0.5), int((p1.y +p2.y)*0.5))

def calcLength(p1,p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def get_eye_open_ratio(frame,landmarks):
        
        ### RIGHT EYE ###
        # horizontal points for right eyes, extreme left and right
        re_left_point = (landmarks.part(36).x, landmarks.part(36).y)
        re_right_point = (landmarks.part(39).x,landmarks.part(39).y)
        #re_hor_line = cv2.line(frame,re_left_point,re_right_point,(0,255,0),2)

        # vertical points for right eye, we have four points here
        re_centre_top = midPoint(landmarks.part(37),landmarks.part(38))
        re_centre_bottom = midPoint(landmarks.part(40),landmarks.part(41))
        #re_vert_line = cv2.line(frame,re_centre_top,re_centre_bottom,(0,0,255),2)

        # finding the lengths of horizontal and vertical line
        re_vert_line_length = calcLength(re_centre_top,re_centre_bottom)
        re_hor_line_length = calcLength(re_left_point,re_right_point)

        # this is the ratio of openness of the eye
        re_eye_open_ratio = (re_hor_line_length/re_vert_line_length)


        ### LEFT EYE ###
        # horizontal points for right eyes, extreme left and right
        le_left_point = (landmarks.part(42).x, landmarks.part(42).y)
        le_right_point = (landmarks.part(45).x,landmarks.part(45).y)
        #le_hor_line = cv2.line(frame,le_left_point,le_right_point,(0,255,0),2)

        # vertical points for right eye, we have four points here
        le_centre_top = midPoint(landmarks.part(43),landmarks.part(44))
        le_centre_bottom = midPoint(landmarks.part(47),landmarks.part(46))
        #le_vert_line = cv2.line(frame,le_centre_top,le_centre_bottom,(0,0,255),2)

        # finding the lengths of horizontal and vertical line
        le_vert_line_length = calcLength(le_centre_top,le_centre_bottom)
        le_hor_line_length = calcLength(le_left_point,le_right_point)

        # this is the ratio of openness of the left eye
        le_eye_open_ratio = (le_hor_line_length/le_vert_line_length)

        # Mark the point on the image
        for i in range(1,27):

            cv2.circle(frame, (landmarks.part(i).x, landmarks.part(i).y), 1, (255,0,0), -1)
        

        return re_eye_open_ratio,le_eye_open_ratio



# Create lists to store time and eye_open_ratio values
time_values = []
eye_open_ratio_values = []

# Create a figure and axis for the plot
fig, ax = plt.subplots()

# Set initial x-axis limits
max_time = 100  # Adjust the value based on your requirements
plt.xlim(0, max_time)

while True:
    
    ret,frame = cap.read()
    frame = cv2.resize(frame,(480,340))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = detector(gray)

    for face in faces:
        x1,y1= face.left(),face.top()
        x2,y2 = face.right(),face.bottom()
        #cv2.rectangle(frame,(x1-30,y1-30),(x2+30,y2+30),(0,0,255),2)

        # finds the 68 facial landmarks
        landmarks = predictor(gray,face)
        
        
        # Bllink Detection
        re_eye_open_ratio,le_eye_open_ratio = get_eye_open_ratio(frame,landmarks)
        eye_open_ratio =  (re_eye_open_ratio+le_eye_open_ratio)*0.5 

        # Append current time and eye_open_ratio to the lists
        time_values.append(len(time_values) + 1)
        eye_open_ratio_values.append(eye_open_ratio)

        if eye_open_ratio > patient_blink_threshold:
            cv2.putText(frame, "BLINKING", (50, 50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 0, 0))

    # Plot eye_open_ratio versus time
    ax.plot(time_values, eye_open_ratio_values, color='red')
    ax.set_xlabel('Time')
    ax.set_ylabel('Closed Eye Ratio')

    # Adjust x-axis limits based on the number of data points
    if len(time_values) > max_time:
        plt.xlim(len(time_values) - max_time, len(time_values))
    else:
        plt.xlim(0, max_time)

    # Update the plot
    plt.pause(0.01)
    ax.cla()  # Clear the plot for the next iteration

        
    cv2.imshow("Feed",frame)

    key = cv2.waitKey(1)
    if key == 27:
        break


cap.release()
cv2.destroyAllWindows()

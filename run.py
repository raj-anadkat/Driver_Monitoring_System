import cv2
import numpy as np
import dlib
import matplotlib.pyplot as plt
import simpleaudio as sa

cap = cv2.VideoCapture(0)

detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

# Change according to the patient
patient_blink_threshold = 4
patient_drowsiness_threshold = 1.7

# asleep conditions
asleep_duration = 0
is_asleep = False

# Variable to track alarm sound playing status
is_alarm_playing = False

buzzer_sound = sa.WaveObject.from_wave_file("warning.wav")



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
        for i in range(1,67):

            cv2.circle(frame, (landmarks.part(i).x, landmarks.part(i).y), 1, (255,0,0), -1)
        

        return re_eye_open_ratio,le_eye_open_ratio

def get_drowsiness_ratio(frame, landmarks):

    # Ratio of how open the mouth is
    #vertical length of distance between lips
    upper_lip_top = (landmarks.part(51).x, landmarks.part(51).y)
    lower_lip_bottom =(landmarks.part(57).x, landmarks.part(57).y)
    #lip_vert_line = cv2.line(frame,upper_lip_top,lower_lip_bottom,(0,0,255),2)

    lip_left = (landmarks.part(48).x, landmarks.part(48).y)
    lip_right = (landmarks.part(54).x, landmarks.part(54).y)
    #lip_hor_line = cv2.line(frame,lip_left,lip_right,(0,0,255),2)

    # finding the lengths of horizontal and vertical line
    lip_vert_line_length = calcLength(upper_lip_top,lower_lip_bottom)
    lip_hor_line_length = calcLength(lip_left,lip_right)


    # usually less than 1.5 if yawning to detect false positives
    drowsiness_ratio = (lip_hor_line_length/lip_vert_line_length)


    return drowsiness_ratio




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
    iris = frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = detector(gray)

    for face in faces:
        x1,y1= face.left(),face.top()
        x2,y2 = face.right(),face.bottom()
        #cv2.rectangle(frame,(x1-30,y1-30),(x2+30,y2+30),(0,0,255),2)

        # finds the 68 facial landmarks
        landmarks = predictor(gray,face)
        # horizontal points for right eyes, extreme left and right
        x_36 = landmarks.part(36).x
        y_36 = landmarks.part(36).y

        x_39 = landmarks.part(39).x
        y_39 = landmarks.part(39).y

        # mid_points
        x_mid_right = round(0.5*(x_36+x_39))
        y_mid_right = round(0.5*(y_36+ y_39))

        # horizontal points for right eyes, extreme left and right
        x_42 = landmarks.part(42).x
        y_42 = landmarks.part(42).y

        x_45 = landmarks.part(45).x
        y_45 = landmarks.part(45).y

        # mid_points
        x_mid_left = round(0.5*(x_42+x_45))
        y_mid_left = round(0.5*(y_42+ y_45))



        
        cv2.circle(iris,(x_mid_right,y_mid_right),1,(255,0,0),2)
        cv2.circle(iris,(x_mid_left,y_mid_left),1,(255,0,0),2)
        
        
        # Blink and Drowsiness Detection 
        re_eye_open_ratio,le_eye_open_ratio = get_eye_open_ratio(frame,landmarks)
        drowsiness_ratio = get_drowsiness_ratio(frame,landmarks)
        #print("Drowsiness Ratio: ",drowsiness_ratio, "Blink Ratio: ",eye_open_ratio)
        eye_open_ratio =  (re_eye_open_ratio+le_eye_open_ratio)*0.5 

        # encoding to 0-1
        if eye_open_ratio > patient_blink_threshold:
            cv2.putText(gray, "BLINKING", (30, 50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 0))
            eye_open_ratio = 1
            asleep_duration += 1
            if asleep_duration >= 5 * 15:  # Assuming the frame rate is 30 fps
                is_asleep = True

        else:
            eye_open_ratio = 0
            asleep_duration = 0
            is_asleep = False

        # 
        if drowsiness_ratio < patient_drowsiness_threshold:
            cv2.putText(gray, "DROWSY", (30, 70), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 0))

        if is_asleep:
            cv2.putText(gray, "Driver Asleep !", (30, 90), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 0), 2)

        if is_asleep and not is_alarm_playing:
            buzzer_sound.play()
            is_alarm_playing = True

        elif not is_asleep and is_alarm_playing:
            is_alarm_playing = False
            
        


                       

        # Append current time and eye_open_ratio to the lists
        time_values.append(len(time_values) + 1)
        eye_open_ratio_values.append(eye_open_ratio)

        

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
    cv2.imshow("gray",gray)

    key = cv2.waitKey(1)
    if key == 27:
        break


cap.release()
cv2.destroyAllWindows()

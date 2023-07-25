import numpy as np
import cv2

def midPoint(p1,p2):
    return (int((p1.x +p2.x)*0.5), int((p1.y +p2.y)*0.5))

def calcLength(p1,p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def get_eye_aspect_ratio(frame,landmarks):
        
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
        re_eye_aspect_ratio = (re_hor_line_length/re_vert_line_length)


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
        le_eye_aspect_ratio = (le_hor_line_length/le_vert_line_length)

        eye_aspect_ratio = (le_eye_aspect_ratio + re_eye_aspect_ratio)/2




        # # # Mark the point on the image
        # for i in range(1,16):
        #     cv2.line(frame, (landmarks.part(i).x, landmarks.part(i).y),((landmarks.part(i+1).x, landmarks.part(i+1).y)), (0,0,255), 1)
        
        # for i in range(37,41):
        #     cv2.line(frame, (landmarks.part(i).x, landmarks.part(i).y),((landmarks.part(i+1).x, landmarks.part(i+1).y)), (0,0,255), 1)
        
        # for i in range(43,47):
        #     cv2.line(frame, (landmarks.part(i).x, landmarks.part(i).y),((landmarks.part(i+1).x, landmarks.part(i+1).y)), (0,0,255), 1)
        

        return eye_aspect_ratio

def get_mouth_aspect_ratio(frame, landmarks):

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
    mouth_aspect_ratio = (lip_hor_line_length/lip_vert_line_length)


    return mouth_aspect_ratio


def get_head_pose(frame,landmarks):

    head_status = "Straight"
        
    model_points = np.array([
    (0.0, 0.0, 0.0),       # Nose tip
    (0.0, -330.0, -65.0),  # Chin
    (-225.0, 170.0, -135.0),  # Left eye corner
    (225.0, 170.0, -135.0),   # Right eye corner
    (-150.0, -150.0, -125.0),  # Left mouth corner
    (150.0, -150.0, -125.0)    # Right mouth corner
])

    # Camera matrix for the intrinsic parameters of the camera
    camera_matrix = np.array([[640.0, 0, 320.0],
                         [0, 640.0, 240.0],
                         [0, 0, 1]], dtype=np.float32)
        

     # Extract the 2D coordinates of the facial landmarks
    image_points = np.array([
            (landmarks.part(30).x, landmarks.part(30).y),  # Nose tip
            (landmarks.part(8).x, landmarks.part(8).y),    # Chin
            (landmarks.part(36).x, landmarks.part(36).y),  # Left eye corner
            (landmarks.part(45).x, landmarks.part(45).y),  # Right eye corner
            (landmarks.part(48).x, landmarks.part(48).y),  # Left mouth corner
            (landmarks.part(54).x, landmarks.part(54).y)   # Right mouth corner
        ], dtype=np.float32)

    # Solve the PnP problem to estimate head pose
    _, rotation_vector, translation_vector = cv2.solvePnP(
            model_points, image_points, camera_matrix, np.zeros(4))

    # Project 3D points to image plane for visualization
    nose_end_point2D, _ = cv2.projectPoints(
            np.array([(0.0, 0.0, 1000.0)]), rotation_vector, translation_vector,
            camera_matrix, np.zeros(4))

    # Draw the 3D pose axis on the face
        

    p1 = (int(image_points[0][0]), int(image_points[0][1]))
    p4 = (int(nose_end_point2D[0][0][0]), int(nose_end_point2D[0][0][1]))
    cv2.line(frame,p1,p4,(0,0,255),3)
    cv2.circle(frame,p4,3,(0,0,255),-1)

    if p1[0] - p4[0] > 150:
        head_status = "Looking Left"
        
    if p1[0] - p4[0] < -150:
        head_status = "Looking Right"
        
    if p1[1] - p4[1] > 80:
        head_status = "Looking Up"
        
    if p1[1] - p4[1] < -80:
        head_status = "Looking Down"

        
    return head_status


def detect_iris(frame,landmarks):
# horizontal points for right eyes, extreme left and right
    x_36,y_36 = landmarks.part(36).x,landmarks.part(36).y
    x_39,y_39 = landmarks.part(39).x,landmarks.part(39).y

    # mid_points
    x_mid_right,y_mid_right = round(0.5*(x_36+x_39)),round(0.5*(y_36+ y_39))

    # horizontal points for right eyes, extreme left and right
    x_42,y_42 = landmarks.part(42).x,landmarks.part(42).y
    x_45,y_45 = landmarks.part(45).x,landmarks.part(45).y

    # mid_points
    x_mid_left,y_mid_left = round(0.5*(x_42+x_45)),round(0.5*(y_42+ y_45))

    cv2.circle(frame,(x_mid_right,y_mid_right),1,(0,0,255),4)
    cv2.circle(frame,(x_mid_left,y_mid_left),1,(0,0,255),4)

    return x_mid_left,x_mid_right
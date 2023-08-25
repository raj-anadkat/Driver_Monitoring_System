#!/usr/bin/env python

import glob
import os
import sys
import numpy as np
import time
import pygame
import carla
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32


IM_WIDTH = 1280
IM_HEIGHT = 720

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# Subscriber Node for DMS Vision topics

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')

        
        self.eye_mode_sub = self.create_subscription(String, 'eye_mode', self.eye_mode_callback,1)
        self.driver_status_sub = self.create_subscription(String,'driver_status',self.driver_status_callback,1)
        self.head_pose_sub = self.create_subscription(String,'head_pose',self.head_pose_callback,1)
        self.eye_aspect_ratio_sub = self.create_subscription(Float32,'eye_aspect_ratio',self.eye_aspect_ratio_callback,1)
        
        self.eye_mode = ""
        self.driver_status = ""
        self.head_pose = ""
        self.eye_aspect_ratio = 0.0

  

    def eye_mode_callback(self, msg):
        self.eye_mode = msg.data
    
    def driver_status_callback(self,msg):
        self.driver_status = msg.data

    def head_pose_callback(self, msg):
        self.head_pose = msg.data
    
    def eye_aspect_ratio_callback(self, msg):
        self.eye_aspect_ratio = msg.data


def process_img(image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (IM_HEIGHT, IM_WIDTH, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    return array

# Initialize the CARLA client
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# Connect to the default map
world = client.get_world()

# Find a vehicle to control
blueprint_library = world.get_blueprint_library()

# Creating a blueprint for the Tesla Model 3
bp = blueprint_library.filter("model3")[0]

# Specify the starting location
start_location = carla.Location(x=11.176284, y=-64.398766, z=0.600000)
start_rotation = carla.Rotation(pitch=0.000000, yaw=179.976562, roll=0.000000)

# Spawn the vehicle at the specified starting location
spawn_point = carla.Transform(start_location, start_rotation)
vehicle = world.spawn_actor(bp, spawn_point)

# Add a camera sensor
camera_bp = blueprint_library.find("sensor.camera.rgb")
camera_bp.set_attribute("image_size_x", f"{IM_WIDTH}")
camera_bp.set_attribute("image_size_y", f"{IM_HEIGHT}")
# camera_bp.set_attribute("fov", "160")

# Initialize pygame
pygame.init()
screen = pygame.display.set_mode((IM_WIDTH, IM_HEIGHT))
font = pygame.font.Font(None, 36)

# intiializing subscriber Node
rclpy.init()
subscriber_node = SubscriberNode()

# Counters for deviations, cooldown, and loop detection
large_deviation_count = 0
last_large_deviation_time = 0
last_eye_blink_time = 0
loop_detected = False
impairment_detected = False
simulation_start_time = time.time()

last_dist_time = 0
distraction_count = 0

blink_count = 0

# Cooldown duration in seconds
cooldown_duration = 3

# Load the buzzer sound
buzzer_sound = pygame.mixer.Sound("buzzer.wav")
blink_sound = pygame.mixer.Sound("blink.mp3")

# array to keep track of position
vehicle_positions = []
lane_locations = []

# Callback function to process the image and display it
def process_image(image):
    global large_deviation_count, last_large_deviation_time, loop_detected, impairment_detected, eye_mode, head_pose, driver_status,eye_aspect_ratio, last_eye_blink_time, last_dist_time,distraction_count,blink_count
    
    
    image_data = process_img(image)
    image_surface = pygame.surfarray.make_surface(image_data.swapaxes(0, 1))
    screen.blit(image_surface, (0, 0))
    
    # Display deviation information
    current_lane = world.get_map().get_waypoint(vehicle.get_location())
    deviation = current_lane.transform.location.distance(vehicle.get_location())
    deviation_text = "Deviation: {:.2f}".format(deviation) + " m"
    deviation_color = (255, 255, 255)  # Default color
    
    if not loop_detected:
        if deviation > 0.9:
            current_time = time.time()
            if current_time - last_large_deviation_time >= cooldown_duration:
                deviation_text += " - Large Deviation!"
                deviation_color = (255, 0, 0)  # Red color for large deviations
                large_deviation_count += 1  # Increment the count of large deviations
                last_large_deviation_time = current_time

                # Play the buzzer sound
                buzzer_sound.play()
            
        if eye_mode == "Closed":
            eye_time = time.time()
            if eye_time - last_eye_blink_time >= 0.5:
                last_eye_blink_time = eye_time
                blink_count +=1
                blink_sound.play()
        
        if driver_status == "Distracted":
            dist_time = time.time()
            if dist_time - last_dist_time >= 3:
                last_dist_time = dist_time
                distraction_count +=1
        
        
        if driver_status == "Asleep":
            buzzer_sound.play()
        
        

        # Display deviation information and count
        deviation_render = font.render(deviation_text, True, deviation_color)
        count_text = "Large Deviations: {}".format(large_deviation_count)
        count_color = (255, 0, 0) if large_deviation_count > 5 else (255, 255, 255)  # Red if > 10, otherwise white
        count_render = font.render(count_text, True, count_color)

        text_color = (255,255,255)
        alert_text_color = (255,0,0)
        eye_mode_text = "Eye Mode: {}".format(eye_mode)

        if eye_mode == "Closed":
            eye_mode_render = font.render(eye_mode_text, True, alert_text_color)
        
        else:
            eye_mode_render = font.render(eye_mode_text, True, text_color)
        
        
        driver_status_text =  "Driver Status: {}".format(driver_status)

        if driver_status == "Attentive":
            driver_status_render = font.render(driver_status_text,True,text_color)
        else:
            driver_status_render = font.render(driver_status_text,True,alert_text_color)


        head_pose_text = "Head Pose: {}".format(head_pose)
        head_pose_render = font.render(head_pose_text, True, text_color)

        eye_aspect_ratio_text = "EAR: {}".format(eye_aspect_ratio)
        eye_aspect_ratio_render = font.render(eye_aspect_ratio_text, True, text_color)

        blink_count_text = "Blinks: {}".format(blink_count)
        blink_count_render = font.render(blink_count_text,True,text_color)

        
        
        screen.blit(deviation_render, (10, 10))
        screen.blit(count_render, (10, 50))
        screen.blit(eye_mode_render, (10, 90))
        screen.blit(driver_status_render, (10, 130))
        screen.blit(head_pose_render, (10, 170))
        screen.blit(eye_aspect_ratio_render, (10, 210))
        screen.blit(blink_count_render, (10, 250))

    
    # Display "Impairment Detected" in the center of the screen if large deviations > 10
    if large_deviation_count > 10 and not loop_detected:
        impairment_text = "Impairment Detected"
        impairment_render = font.render(impairment_text, True, (255, 0, 0))  # Red color
        
        # Calculate the position to center the text
        text_width, text_height = font.size(impairment_text)
        text_x = (IM_WIDTH - text_width) // 2
        text_y = (IM_HEIGHT - text_height) // 2
        
        screen.blit(impairment_render, (text_x, text_y))
        
        # Set impairment_detected flag to True
        impairment_detected = True
    
    # Display "Loop detected ending the test" and test result
    if loop_detected:
        loop_detected_text = "Loop detected ending the test"
        loop_detected_render = font.render(loop_detected_text, True, (0, 0, 0))  # white color
        
        # Calculate the position to center the text
        loop_text_width, loop_text_height = font.size(loop_detected_text)
        loop_text_x = (IM_WIDTH - loop_text_width) // 2
        loop_text_y = (IM_HEIGHT - loop_text_height) // 2
        
        screen.blit(loop_detected_render, (loop_text_x, loop_text_y))
        
        # Display the test result
        result_text = "Passed the test" if not impairment_detected else "Failed the test"
        result_render = font.render(result_text, True, (0, 255, 0) if not impairment_detected else (255, 0, 0))
        
        # Calculate the position to center the text
        result_text_width, result_text_height = font.size(result_text)
        result_text_x = (IM_WIDTH - result_text_width) // 2
        result_text_y = loop_text_y + loop_text_height + 10  # Display below the "Loop detected" message
        
        screen.blit(result_render, (result_text_x, result_text_y))
    
    pygame.display.flip()

# Add a camera sensor on the vehicle
sensor = world.spawn_actor(camera_bp, carla.Transform(carla.Location(x=2.5, z=1.2)), attach_to=vehicle)
sensor.listen(process_image)

try:
    while not loop_detected:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()
        
        eye_mode = subscriber_node.eye_mode
        driver_status = subscriber_node.driver_status
        head_pose = subscriber_node.head_pose
        eye_aspect_ratio = round(subscriber_node.eye_aspect_ratio,2)

        rclpy.spin_once(subscriber_node, timeout_sec=0.01)  # Adjust the timeout as needed

        # Get keyboard input
        keys = pygame.key.get_pressed()
        if keys[pygame.K_w]:
            throttle = 1.0
        elif keys[pygame.K_s]:
            throttle = -1.0
        else:
            throttle = 0.0

        if keys[pygame.K_a]:
            steer = -0.5
        elif keys[pygame.K_d]:
            steer = 0.5
        else:
            steer = 0.0

        # Apply control to the vehicle
        control = carla.VehicleControl()
        control.throttle = throttle
        control.steer = steer
        vehicle.apply_control(control)
        
        # Check if the vehicle is close to the starting location and sufficient time has elapsed
        current_location = vehicle.get_location()
        if (current_location.distance(start_location) < 5.0) and (time.time() - simulation_start_time > 30.0):
            loop_detected = True

            print("********************    RESULTS    ************************")
            print("Loop detected, stopping the vehicle.")
            print("Number of Major Lane Deviations: ",large_deviation_count)
            print("Number of Distractions: ",distraction_count)
            print("Number of Blinks: ",blink_count)
            print("Average Eye Aspect Ratio: ", eye_aspect_ratio)
            result_text = "Pass" if large_deviation_count <10 else "Fail"
            print("Test Result: ",result_text)
            print("***********************************************************")
            vehicle.apply_control(carla.VehicleControl())  # Stop the vehicle

        current_lane = world.get_map().get_waypoint(vehicle.get_location())
        lane_location = current_lane.transform.location
        vehicle_positions.append(current_location)
        lane_locations.append(lane_location)


finally:
    # # Wait for the result to be displayed for 5 seconds
    # result_display_start_time = time.time()
    # while time.time() - result_display_start_time < 5:
    #     pygame.event.pump()
    #     pygame.display.flip()
    
    sensor.destroy()
    vehicle.destroy()
    pygame.quit()

    x_positions = [pos.x for pos in vehicle_positions]
    y_positions = [pos.y for pos in vehicle_positions]

    x_lane_positions = [pos.x for pos in lane_locations]
    y_lane_positions = [pos.y for pos in lane_locations]

    # plotting the trajectory
    plt.figure(figsize=(10,6))
    plt.plot(x_positions,y_positions,marker = "o",label = "Vehicle Trajectory")
    plt.plot(x_lane_positions,y_lane_positions,marker = "x", linestyle="--", color = "red")
    plt.title("Vehicle Trajectory")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    plt.show()

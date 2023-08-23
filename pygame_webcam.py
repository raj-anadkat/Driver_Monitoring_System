#!/usr/bin/env python

import glob
import os
import sys
import numpy as np
import cv2
import time

IM_WIDTH = 1280
IM_HEIGHT = 720
WEBCAM_WIDTH = 320
WEBCAM_HEIGHT = 240

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import pygame

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

# Adding a spawn point
spawn_point = carla.Transform(carla.Location(x=11.176284, y=-64.398766, z=0.600000), carla.Rotation(pitch=0.000000, yaw=179.976562, roll=0.000000))

vehicle = world.spawn_actor(bp, spawn_point)

# Add a camera sensor
camera_bp = blueprint_library.find("sensor.camera.rgb")
camera_bp.set_attribute("image_size_x", f"{IM_WIDTH}")
camera_bp.set_attribute("image_size_y", f"{IM_HEIGHT}")

# Add a lane invasion sensor
lane_invasion_bp = blueprint_library.find("sensor.other.lane_invasion")

# Initialize pygame
pygame.init()
screen = pygame.display.set_mode((IM_WIDTH, IM_HEIGHT))
font = pygame.font.Font(None, 36)

# Initialize the webcam
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FPS, 20)

# Callback function to process the image and display it
def process_image(image):
    image_data = process_img(image)
    image_surface = pygame.surfarray.make_surface(image_data.swapaxes(0, 1))
    screen.blit(image_surface, (0, 0))

    # Display lane deviation count
    lane_deviation_text = font.render(f"Lane Deviations: {lane_deviation_count}", True, (255, 255, 255))
    screen.blit(lane_deviation_text, (10, 10))

    # Capture a frame from the webcam
    ret, frame = cap.read()
    if ret:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        webcam_surface = pygame.surfarray.make_surface(frame.swapaxes(0, 1))
        # Resize the webcam image to fit the small box
        webcam_surface = pygame.transform.scale(webcam_surface, (WEBCAM_WIDTH, WEBCAM_HEIGHT))
        screen.blit(webcam_surface, (IM_WIDTH - WEBCAM_WIDTH, 0))

    pygame.display.flip()

# Callback function for lane invasion events
def process_lane_invasion(event):
    global lane_deviation_count, prev_count, cooldown_timer

    # Check if the cooldown timer has expired
    if cooldown_timer <= 0:
        lane_deviation_count += 1
        prev_count = 1
        cooldown_timer = cooldown_duration  # Reset cooldown timer
        print("Lane invasion detected!")
        print(f"Number of Lane Deviations: {lane_deviation_count}")

# Add a camera sensor on the vehicle
sensor = world.spawn_actor(camera_bp, carla.Transform(carla.Location(x=2.5, z=1.2)), attach_to=vehicle)
sensor.listen(process_image)

# Add a lane invasion sensor on the vehicle
lane_invasion_sensor = world.spawn_actor(lane_invasion_bp, carla.Transform(), attach_to=vehicle)
lane_invasion_sensor.listen(process_lane_invasion)

# Counter for lane deviation events
lane_deviation_count = 0
prev_count = 0

# Cooldown variables
cooldown_timer = 0
cooldown_duration = 3  # Cooldown duration in seconds

try:
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

        # Update the cooldown timer
        cooldown_timer -= 1 / pygame.time.get_ticks()

        # Get keyboard input
        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP]:
            throttle = 1.0
        elif keys[pygame.K_DOWN]:
            throttle = -1.0
        else:
            throttle = 0.0

        if keys[pygame.K_LEFT]:
            steer = -0.5
        elif keys[pygame.K_RIGHT]:
            steer = 0.5
        else:
            steer = 0.0

        # Apply control to the vehicle
        control = carla.VehicleControl()
        control.throttle = throttle
        control.steer = steer
        vehicle.apply_control(control)

finally:
    sensor.destroy()
    lane_invasion_sensor.destroy()
    vehicle.destroy()
    cap.release()  # Release the webcam
    pygame.quit()
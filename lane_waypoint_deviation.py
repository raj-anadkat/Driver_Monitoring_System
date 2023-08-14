#!/usr/bin/env python

import glob
import os
import sys
import numpy as np

IM_WIDTH = 1280
IM_HEIGHT = 720

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
# camera_bp.set_attribute("fov", "160")

# Initialize pygame
pygame.init()
screen = pygame.display.set_mode((IM_WIDTH, IM_HEIGHT))
font = pygame.font.Font(None, 36)

# Callback function to process the image and display it
def process_image(image):
    image_data = process_img(image)
    image_surface = pygame.surfarray.make_surface(image_data.swapaxes(0, 1))
    screen.blit(image_surface, (0, 0))
    
    # Display deviation information
    current_lane = world.get_map().get_waypoint(vehicle.get_location())
    deviation = current_lane.transform.location.distance(vehicle.get_location())
    deviation_text = "Deviation: {:.2f} m".format(deviation)
    deviation_color = (255, 255, 255)  # Default color
    
    if deviation > 0.9:
        deviation_text += " - Large Deviation!"
        deviation_color = (255, 0, 0)  # Red color for large deviations
    
    text_render = font.render(deviation_text, True, deviation_color)
    screen.blit(text_render, (10, 10))
    
    pygame.display.flip()

# Add a camera sensor on the vehicle
sensor = world.spawn_actor(camera_bp, carla.Transform(carla.Location(x=2.5, z=1.2)), attach_to=vehicle)
sensor.listen(process_image)

try:
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

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
    vehicle.destroy()
    pygame.quit()

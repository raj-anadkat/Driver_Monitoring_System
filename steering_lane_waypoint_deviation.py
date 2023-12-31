#!/usr/bin/env python
import glob
import os
import sys
import numpy as np
import time
import pygame
import carla
from configparser import ConfigParser
import math

IM_WIDTH = 1280
IM_HEIGHT = 720

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


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
pygame.joystick.init()
screen = pygame.display.set_mode((IM_WIDTH, IM_HEIGHT))
font = pygame.font.Font(None, 36)

# Joystick Initialization
_joystick = pygame.joystick.Joystick(0)
_joystick.init()
_parser = ConfigParser()
_parser.read('wheel_config.ini')
_steer_idx = int(
_parser.get('G29 Racing Wheel', 'steering_wheel'))
_throttle_idx = int(
_parser.get('G29 Racing Wheel', 'throttle'))
_brake_idx = int(_parser.get('G29 Racing Wheel', 'brake'))
_reverse_idx = int(_parser.get('G29 Racing Wheel', 'reverse'))
_handbrake_idx = int(_parser.get('G29 Racing Wheel', 'handbrake'))


# Counters for deviations, cooldown, and loop detection
large_deviation_count = 0
last_large_deviation_time = 0
loop_detected = False
impairment_detected = False
simulation_start_time = time.time()

# Number of tries allowed
impairment_tolerance = 10 

# Cooldown duration in seconds
cooldown_duration = 3

# Load the buzzer sound
buzzer_sound = pygame.mixer.Sound("buzzer.wav")

# steering wheel function
def _parse_vehicle_wheel():
    numAxes = _joystick.get_numaxes()
    jsInputs = [float(_joystick.get_axis(i)) for i in range(numAxes)]

    jsButtons = [float(_joystick.get_button(i)) for i in
                     range(_joystick.get_numbuttons())]

    # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
    # For the steering, it seems fine as it is
    K1 = 1.0  # 0.55
    steerCmd = K1 * math.tan(1.1 * jsInputs[_steer_idx])

    K2 = 1.6  # 1.6
    throttleCmd = K2 + (2.05 * math.log10(
            -0.7 * jsInputs[_throttle_idx] + 1.4) - 1.2) / 0.92
    if throttleCmd <= 0:
        throttleCmd = 0
    elif throttleCmd > 1:
        throttleCmd = 1

    brakeCmd = 1.6 + (2.05 * math.log10(
            -0.7 * jsInputs[_brake_idx] + 1.4) - 1.2) / 0.92
    if brakeCmd <= 0:
        brakeCmd = 0
    elif brakeCmd > 1:
        brakeCmd = 1
    
    
    #hand_brake = bool(jsButtons[self._handbrake_idx])
    reverse = bool(jsButtons[_reverse_idx])
    
    return steerCmd,brakeCmd,throttleCmd,reverse


# Callback function to process the image and display it
def process_image(image):
    global large_deviation_count, last_large_deviation_time, loop_detected, impairment_detected
    
    image_data = process_img(image)
    image_surface = pygame.surfarray.make_surface(image_data.swapaxes(0, 1))
    screen.blit(image_surface, (0, 0))
    
    # Display deviation information
    current_lane = world.get_map().get_waypoint(vehicle.get_location())
    deviation = current_lane.transform.location.distance(vehicle.get_location())
    deviation_text = "Deviation: {:.2f}".format(deviation)
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
        
        # Display deviation information and count
        deviation_render = font.render(deviation_text, True, deviation_color)
        count_text = "Large Deviations: {}".format(large_deviation_count)
        count_color = (255, 0, 0) if large_deviation_count > 10 else (255, 255, 255)  # Red if > 10, otherwise white
        count_render = font.render(count_text, True, count_color)
        
        screen.blit(deviation_render, (10, 10))
        screen.blit(count_render, (10, 50))
    
    # Display "Impairment Detected" in the center of the screen if large deviations > 10
    if large_deviation_count > impairment_tolerance and not loop_detected:
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

        
        # getting inputs from the wheel
        steer,brake,throttle,reverse = _parse_vehicle_wheel()
                
        # Apply control to the vehicle
        control = carla.VehicleControl()
        control.throttle = throttle
        control.steer = steer
        control.brake = brake
        


        vehicle.apply_control(control)
        
        # Check if the vehicle is close to the starting location and sufficient time has elapsed
        current_location = vehicle.get_location()
        if (current_location.distance(start_location) < 5.0) and (time.time() - simulation_start_time > 30.0):
            loop_detected = True
            print("Loop detected, stopping the vehicle.")
            vehicle.apply_control(carla.VehicleControl())  # Stop the vehicle

finally:
    # Wait for the result to be displayed for 5 seconds
    result_display_start_time = time.time()
    while time.time() - result_display_start_time < 5:
        pygame.event.pump()
        pygame.display.flip()

    sensor.destroy()
    vehicle.destroy()
    pygame.quit()



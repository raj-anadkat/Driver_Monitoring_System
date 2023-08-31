# Driver Monitoring System
Guardian Angel - The goal is to have continuous cognitive capability assessment for early-stage dementia and post-stroke patients. We will do this within the driving activity context.
So continually monitor the driving skill, attention and vital signs of the patient while they are driving by putting sensors in the driving simulator rig.
The outcome will let the physician know when and how often the patient had “Cognitive Impairment Events” which can serve as indicators for cognitive decline.

# Vision Stack:

Features: 
Blink Detection: The system is accurately able to detect whether the driver blinks or not, what the average eye aspect ratio is and the average blink frequency.

Sleep detection: If the eyes of the driver are closed for more than a present duration, the system detects that the driver is asleep

Drowsiness Detection: The system detects yawning by looking at the mouth aspect ratio. Along with that, the eye aspect ratio tends to go down while someone is drowsy. With these inputs, we can determine if the patient is drowsy.

Talk detection: Rapid changes in mouth aspect ratio can be useful to detect if the patient is talking. This can be an important feature to detect if the patient is distracted.

Head Pose Estimation: With the help of facial landmarks, we can find the direction where the head is pointing towards. Then using projective geometry and camera intrinsics, we can determine where the head is pointing towards. This helps determine if the driver is looking straight, left, right, up or down.

# Health Stack

Features:
average BPM: Every driver may have a different heart rate depending on their situation, someone might be driving just after going to the gym and may have a different average. It is important to take into account the average and set this as a benchmark to see instantaneous spikes.

Instantaneous heart Rate: Some events like stress, or road rage incidents may cause an instantaneous spike in the heart rate and this may lead to impairments. Thus an accurate way to measure these spikes is crucial.

spO2 levels: It may or may not be helpful to detect blood oxygen levels. A decrease in spO2 levels may lead to cognitive impairments.

# Simulator Setup

We integrated the Logitech G290 steering wheel and pedals with the CARLA UE4 simulator with some modifications. Despite the absence of native force feedback support on Linux, we leveraged external packages to seamlessly integrate force feedback, thereby delivering lifelike steering experience. Presently, our setup maintains an average frame rate of 25-30 FPS, even while running the Vision stack alongside CARLA UE4. Below is an illustration of our configuration:

Creating Scenarios:

With the help of Carla Scenario Runner, we can create custom scenarios  to monitor braking distance, add pedestrian scenarios, cars etc. We can detect collisions, average distance from a car , lane switching, acceleration and speed limit violations to track driving activity. We can also track deviation from the lane as a feature.

# Installation Steps
1) Download the facial landmarks .dat file and add it in the same directory:
Link to the file: https://drive.google.com/file/d/1HFpq7n17R1W73vxMtAIWOL2xDGQpKe5C/view?usp=sharing
2) Run the run.py file. You may need to modify the patient thresholds.

# Results
<iphy src="https://github.com/raj-anadkat/Driver_Monitoring_System/assets/109377585/4dd7b747-d5a8-4117-924e-3eb312d1a053"/>
<img src="https://github.com/raj-anadkat/Driver_Monitoring_System/assets/109377585/c7cc0fa7-cbf9-4bdc-b95f-dbd34d57933c" width="500"/>
<img src="https://github.com/raj-anadkat/Driver_Monitoring_System/assets/109377585/88d36319-1109-498c-808f-4a482e5c4be7" width="500"/>
<img src="https://github.com/raj-anadkat/Driver_Monitoring_System/assets/109377585/837d6886-b0e0-4ac1-b834-a46e539432da" width="500"/>


## Simulation
https://github.com/raj-anadkat/Driver_Monitoring_System/assets/109377585/289df16f-ebee-4da4-ad91-4d6f785f7e2c




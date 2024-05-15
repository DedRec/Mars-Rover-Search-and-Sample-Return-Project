# Mars-Rover-Search-and-Sample-Return-Project

## Introduction
This project aims to develop an autonomous navigation system for a rover using perception and decision-making algorithms. The rover's goal is to navigate through an environment, avoiding obstacles, locating rock samples, and collecting them.

## Files Description
- `decision.py`: Contains the decision-making logic for throttle, brake, and steering commands based on perception data.
- `drive_rover.py`: The main script to run the rover's autonomous navigation. It interfaces with the simulator and executes perception and decision steps.
- `perception.py`: Implements perception functions such as color thresholding, perspective transformation, and rock sample detection.
- `supporting_functions.py`: Includes supporting functions for telemetry processing, image conversion, and output image creation.

## Usage
1. **Setting Up Environment**: Ensure the necessary dependencies are installed.
2. **Running the Rover**: Execute `drive_rover.py` to start the rover's autonomous navigation.
3. **Interacting with Simulator**: Use the simulator to observe the rover's behavior and performance.
4. **Debugging**: Optionally, enable debugging mode in `drive_rover.py` for visualizing intermediate pipeline images.

## Code Overview
- **Decision Making**: The `decision.py` file contains the decision-making logic based on perception data. It determines throttle, brake, and steering commands to navigate the rover.
- **Perception**: Perception functions in `perception.py` process camera images, perform color thresholding, identify obstacles and rock samples, and transform coordinates.
- **Supporting Functions**: The `supporting_functions.py` includes utilities for telemetry processing, image conversion, and creating output images.

## How It Works
1. **Perception Step**: Camera images are processed to identify navigable terrain, obstacles, and rock samples. Perspective transformation and color thresholding are applied.
2. **Decision Step**: Based on perception results, decision logic determines rover actions such as throttle, brake, and steering.
3. **Telemetry Processing**: Telemetry data from the simulator is processed to update rover state including position, velocity, and sensor readings.
4. **Autonomous Navigation**: The rover autonomously navigates through the environment, avoiding obstacles, collecting rock samples, and updating its world map.

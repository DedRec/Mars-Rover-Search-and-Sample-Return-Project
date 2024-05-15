# Mars-Rover-Search-and-Sample-Return-Project

## Overview

This project involves developing a navigation system for a rover to autonomously explore an unknown terrain, identify obstacles, locate and collect rock samples, and map the environment using onboard sensors and cameras. The project consists of several Python files that implement various functionalities required for rover navigation and perception.

## Files

### perception.py

- **Description:** This file contains functions related to image processing and perception, including color thresholding, perspective transformation, obstacle detection, rock sample identification, and conversion between image coordinates and world coordinates.
- **Functions:**
  - `color_thresh_decision`: Performs color thresholding to identify navigable terrain.
  - `color_thresh`: Performs color thresholding to identify obstacles.
  - `color_thresh_color_img`: Returns a colored image with thresholded pixels highlighted.
  - `rover_coords`: Converts image coordinates to rover-centric coordinates.
  - `to_polar_coords`: Converts rover-centric coordinates to polar coordinates.
  - `rotate_pix`: Applies rotation to pixel coordinates.
  - `translate_pix`: Applies translation to pixel coordinates.
  - `pix_to_world`: Converts rover-centric pixel coordinates to world coordinates.
  - `perspect_transform`: Performs perspective transformation on images.
  - `find_rocks`: Identifies rock samples in images.
  - `divideConquer`: Implements a divide and conquer strategy for perception.

### supporting_functions.py

- **Description:** This file contains additional functions to support rover operations, including telemetry data processing and image display.
- **Functions:**
  - `convert_to_float`: Converts telemetry strings to float values.
  - `update_rover`: Updates the Rover's state based on telemetry data.
  - `create_output_images`: Generates output images for display based on rover's worldmap and vision image.

## Usage

1. Ensure all dependencies are installed, including NumPy, OpenCV, PIL, and Matplotlib.
2. Run the main script to start the rover navigation system.
3. Monitor the rover's behavior and performance using the provided visualization tools.
# Sample & Return Rover Computer Vision Project

This project focuses on implementing computer vision techniques for robotics, particularly in the context of building a Sample & Return Rover in simulation. The primary objective is to control the rover using images streamed from a camera mounted on it. The project involves autonomous mapping and navigation based on an initial map of the environment. The mapping component constitutes a significant portion of the project's complexity.

![Rover Image](https://user-images.githubusercontent.com/89746218/205963130-18d953fb-f137-4544-b7fa-65c3de862438.png)

## Phase One Objectives

1. Develop a pipeline capable of mapping at least 40% of the environment with 60% fidelity. The pipeline should differentiate between navigable terrain, obstacles, and rock samples on the map image.
2. Locate at least one rock in the environment.
3. Implement a debugging mode where each step of the pipeline is illustrated along with the corresponding vehicle operation.

![Pipeline Image](https://user-images.githubusercontent.com/89746218/205963663-9701bec3-f35e-475e-90c4-ee565323905c.png)

## How to Run on Simulator

The project code includes necessary Python files. `drive_rover.py` facilitates autonomous navigation of the rover environment and invokes functions from `perception.py` and `decision.py`. All functionalities defined in the Jupyter notebook are encapsulated in `perception.py`.

To execute the project on the simulator:

1. Open a Linux terminal.
2. Run the following command to enable debugging mode using the keyboard library (must be root to use this library):

```bash
su root
```

3. Then execute:

```bash
python drive_rover.py
```

Launch the simulator and select "Autonomous Mode". The rover will initiate autonomous navigation, displaying a map indicating navigable terrain, obstacles, and rock sample locations. To activate debugging mode, showcasing each step of the pipeline with the corresponding vehicle operation, press the letter 'm'.

## Debugging Mode Illustration

![Debugging Mode](https://user-images.githubusercontent.com/89746218/206920089-a868fdc6-fbd9-48b6-98fb-43eb9f19f8bb.png)

## Mapping Results

### Testing 1
![Test 1](https://user-images.githubusercontent.com/89746218/206920139-ad4c6c45-3a39-43ec-ba51-bcf9670af4d2.png)

### Testing 2
![Test 2](https://user-images.githubusercontent.com/89746218/206920122-957b4868-ba46-4f97-877c-b78cab61acc9.png)



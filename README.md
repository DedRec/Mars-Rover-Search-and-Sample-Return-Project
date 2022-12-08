# Computer Vision Project

In this project, we’ll do computer vision for robotics. 
We are going to build a Sample & Return Rover in 
simulation. Mainly, we’ll control the robot from images 
streamed from a camera mounted on the robot. The 
project aims to do autonomous mapping and 
navigation given an initial map of the environment. 
Realistically speaking, the hard work is done now that 
you have the mapping component! You will have the 
option to choose whether to send orders like the 
throttle, brake, and steering with each new image the 
rover's camera produces

![image](https://user-images.githubusercontent.com/89746218/205963130-18d953fb-f137-4544-b7fa-65c3de862438.png)

## In phase one

1) pipeline should be able to map at least 40% of the environment at 60% fidelity. It should 
repaint the map image to distinguish between navigable terrain, obstacles and rock samples
2) locate at least one rock in the environment
3) implement debugging mode where each step of your pipeline is 
illustrated with the vehicle operation

![image](https://user-images.githubusercontent.com/89746218/205963663-9701bec3-f35e-475e-90c4-ee565323905c.png)

## To Run On Simulator 
In code file you will find python files needed, drive_rover.py is needed to navigate the rover environment in autonomous mode, it alse calls functions from within perception.py and decision.py.
everything in the jupyter notebook is defined in perception.py

Run the following command on cmd 
```
python drive_rover.py
```
Then launch the simulator and choose choose "Autonomous Mode", Rover will start by driving itself and showing a map showing navigable terrain, obstacles and rock sample locations

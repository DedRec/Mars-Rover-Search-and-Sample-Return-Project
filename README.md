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
In code file you will find python files needed, drive_rover.py is needed to navigate the rover environment in autonomous mode, 
it also calls functions from within perception.py and decision.py.
Everything in the jupyter notebook is defined in perception.py

Run the following command on linux terminal 
this command will aid to use the keyboard library for debugging mode
```
su root
```
then
```
python drive_rover.py
```
Then launch the simulator and choose choose "Autonomous Mode", Rover will start by driving itself and showing a map that shows navigable terrain, obstacles and rock 
sample locations, to view debugging mode where each step of the pipeline is illustrated with the vehicle operation
click on letter 'm'

## Debugging mode 
![image](https://user-images.githubusercontent.com/89746218/206920089-a868fdc6-fbd9-48b6-98fb-43eb9f19f8bb.png)

## Mapping
#### testing 1
![image](https://user-images.githubusercontent.com/89746218/206920139-ad4c6c45-3a39-43ec-ba51-bcf9670af4d2.png)

#### testing 2

![image](https://user-images.githubusercontent.com/89746218/206920122-957b4868-ba46-4f97-877c-b78cab61acc9.png)




## Project: Search and Sample Return

---


**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./misc/thresholded.jpg
[image2]: ./calibration_images/example_rock1.jpg 

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.

To identify terrain, rocks and obstacles we are just thresholding the camera images creating an image mask.
In the notebook (on the project I exprerimented more) I used
- to threshold terrain rgb values greater than (160,160,160), although you can go lower especially in the blue component to get more of those saded sand regions
- for the obstacles I used all rgb values between (5,5,5) and (160,160,160). The reason for the lower limit is because in the birds view, a portion of the mask is not viewable and as a result of the transormation its pure black, so I exclude those pixels since they are not obstacles but absense of information (in the project I limit the region of interest so this is not neccessary at all)
- I used HSV space to threshold the rock , with low value (20,100,100) and high (25,255,255), but RGB solution work fine too.

![image and thresholds][image1]

#### 2. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 

- Most of the work in process image is as explained in the lectures. The extra step is storing the results in an images (worldmap) using each channel to store information about obstacles, rocks and terrain.
Every pixel identified as either had the corresponding channel increase its value by one. 
If a pixel at (x,y) had a value larger than zero in both the obstacle and the terrain channel, then we "believe" the channel with the higher value.

- An extra step (which i used only in the project and not the notebook) is update the worldmap, only when roll and pitch are low (between [0,1] or [359, 360) ). This is because when the vehicle, for example, brakes, or is elevated in some rock, the coordinates we get from the camera do not correspond to the real flat world map.
We could try to project them back to the x,y plane , but its too much computation work and not very useful


### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

- the perception step is pretty much the same as above. I just used little different rbg values (to get more or less of the obstacles and terrain and to see if it alters the quality of the driving.)

- To make the decision step simpler and easier to experiment this I extended the RoverState class, actually created a separate file for it, and put all the logic there.

- The main idea is the RoverState, to be a state machine. You have a "stack" that on top has the command that has to be executed. So in each cycle , the function corresponding to the top state is executed. 
If I want , for example, to suspend what the Robot is doing to collect a stone, a command is placed on top of the stack, so another function is executed and when the job is finished , the command is popped from the stack.

- That way you can give a series of commands to the robot (go 5 meters ahead, then turn 20 degrees... etc)
My initial idea was to keep track of the the map created and tell the robot to go from point A to point B.
I first created the algorithm to find me the path, but the harder part was selecting the destination point, dealing with the inaccuracies of the reading.. etc. It was much harder to go that way, so after 4 days I resigned from that idea, and headed to the simpler "crawl the wall" method.

My robot uses 5 states basically. 
- waiting-command (which just gets the top command from the stack and uses it as the current robot mode)
- finished-command (which pops the the top of the stack)
- mapping (which does all the navigation)
- collecting (which is executed when we see a rock)
- unstuck (when the robot has not moved for some time, it tries to free the robot)

#### Mapping 

I basically crawl the left wall using the following
 - restrict vision data to small rectangular around the robot. The wider the rectangle the more inaccurate the information
 - from the obstacles channel, from the robot's camera view , I monitor the left bottom window. I want it to contain no obstacle pixels. This results that i do not go to close to the wall
 - I also use from the same channel the middle region. That region I want it to have obstacle pixels. That means the robot is following the wall. If it drops bellow a value, it means no obstacle on the left, so the robot should turn that way.
 - I also monitor front pixels to avoid colision. When I am about to crash, I stop and turn right (to continue the "crawl from left to right)
 - I do not used fixed values for steering  but rather random number in some range. This is to avoid the robot falling into loops.

 #### Collecting

 - From the rock's channel from image data, I transform that array to polar coordinates and monitor for nonzero angles. A non zero angle means there is a rock in the field of view of the robot. I use that angle to navigate the robot to the rock. 
 - Since I want to continue crawling from left to right , I consider only angles that are in [-10,90] , that is positive, to the left of the robot , or slightly to the right, to avoid the robot jumping to the other side of the road for a rock

 #### Unstuck

 - I created 7 different strategies to unstuck the robot. Turning, hitting the throttle, going back.
 - The key is not to apply them in order. I you do , you risk stucking in a loop.
 - The robot will unstuck but then will most likely go around the same place, and then apply the same bad manouevre that was not effective in the first place.

 - To avoid that I randomly select and apply one. Ofcourse this is not very realistic to have at every iteration another strategy and could be vastly improved, but it does the job for the project



#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: Resolution: 800x600, Graphics quality: Fantastic, FPS is about 50.**

My rover map 97% of the environment with 67.7% fidelity against the ground truth within 15 min, but that may very because of the randomized way the robot picks the angles. I have not programmed it to return to base. It was not a requirement and I run out of time.

There are tons of ways to improve this
- the most important think is I am not using prior knowledge of the environment. I keep track of it but not using it. So a very basic improvement would be to check both the vision data and the visited map data.
- there are spots my robot will not go. Because I programmed it to be not very close to the wall, to avoid many collisions, it will fail to enter some tiny spots there are in the map. Its a choice I mad.
- The robot could stop and look around when it senses open space (no obstacles around) to improve the understanding of the surrounding. Ofcourse this would have to be augmented with the first step ( use knowledge of the environement for better navigation)
- Path planning ofcourse would be the next step



![alt text][image2]



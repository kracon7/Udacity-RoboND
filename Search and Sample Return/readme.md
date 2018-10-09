## Project: Search and Sample Return
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

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

[image1]: ./p1.png

[image2]: ./p2.png

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.

For image analysis.
First define a funciton `perspect_transform` for perspective transform and use openCV to calculate the transform between source points and destination points
```python
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    return warped
```

Then I defined a `color_thresh` function to generate a binary image where pixel value larger than all three `rgb_thresh` index will be 1, otherwise this pixel will be given 0 
Similarly we can define the function `find_rocks` to identify the samples by modifying different `rgb_thresh` in function `color_thresh`. By changing arguement `Above` between Ture and False, I can set the upper and lower bound for rock threshold. Here the default upper and lower threshold are [250, 250, 60] and [100, 100, 0]
```python
def color_thresh(img, rgb_thresh=(160, 160, 160), Above = True):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    if Above:
        above_thresh = (img[:,:,0] >= rgb_thresh[0]) \
                    & (img[:,:,1] >= rgb_thresh[1]) \
                    & (img[:,:,2] >= rgb_thresh[2])
        # Index the array of zeros with the boolean array and set to 1
        color_select[above_thresh] = 1
    else:
        below_thresh = (img[:,:,0] <= rgb_thresh[0]) \
                    & (img[:,:,1] <= rgb_thresh[1]) \
                    & (img[:,:,2] <= rgb_thresh[2])
        color_select[below_thresh] = 1
    
    # Return the binary image
    return color_select
```
Similarly we can define the function `find_rocks` to identify the samples by modifying different `rgb_thresh` in function `color_thresh`.  




#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 

The `process_image` maps identified pixels navigable terrain, obstacles and rock samples into a worldmap. This is done in by the following steps.

First I define source and destination points for perspective transform.
Then I apply perspective transform to generate a top-down view of the environment which is later used to identify the navigable terrian, obstacles and rocks. `perspect_transform()` generate a image named `warped` and by applying different color threshold I can get `nav_threshold`, `obs_threshold`, `rock_threshold` representing the position of navigable terrain, obstacles and sample in rover coordinate system.
```python
    nav_threshed = color_thresh(warped)
    obs_threshed = color_thresh(warped, Above = False)
    rock_threshed = find_rocks(warped)
    
    nav_xpix, nav_ypix = rover_coords(nav_threshed)
    obs_xpix, obs_ypix = rover_coords(obs_threshed)
    rock_xpix, rock_ypix = rover_coords(rock_threshed)
```
After that I transform them into world coordinates via `pix_to_world()` and use the transformed position to update the world map.  
```python
    nav_x_world, nav_y_world = pix_to_world(nav_xpix, nav_ypix, xpos, ypos, yaw, world_size, scale)
    obs_x_world, obs_y_world = pix_to_world(obs_xpix, obs_ypix, xpos, ypos, yaw, world_size, scale)
    rock_x_world, rock_y_world = pix_to_world(rock_xpix, rock_ypix, xpos, ypos, yaw, world_size, scale)
    
    data.worldmap[obs_y_world, obs_x_world, 0] += 255
    data.worldmap[rock_y_world, rock_x_world, 1] += 255
    data.worldmap[nav_y_world, nav_x_world, 2] += 255
```
Here's an example of this process
![alt text][image1]


### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

In `perception.py` I defined same color threshold function as in the notebook in order to identify navigable terrain, obstacles and samples.
Then In `perception_step()`, pass the position and yaw from `RoverState()` into the function:
```python
    xpos = Rover.pos[0]
    ypos = Rover.pos[1]
    yaw = Rover.yaw
    image = Rover.img
```
Then perform perspective transform of camera image to `Rover.vision` and `Rover.world_map`. The procedure is similar to what I did in the notebook. 
One major difference here is how I update the navigable terrain and obstacle in the world map. I found that everytime the rover brake after it detect a rock, the fidelity drops significantly. It means that updating worldmap while the pitch and row is large is not a good idea. So I only update worldmap when row and pitch is between -0.5 and 0.5. It worked perfectly. The fidelity increased from around 50% to 75%. 
```python
if Rover.roll <= 0.5 or Rover.roll >= 359.5:
    if Rover.pitch <= 0.5 or Rover.pitch >= 359.5:
        Rover.worldmap[obs_y_world, obs_x_world, 0] += 255
        Rover.worldmap[rock_x_world, rock_y_world, :] += 255
        Rover.worldmap[nav_y_world, nav_x_world, 2] += 255
```



In `decision_step()` 
I considered two cases in 'forward' mode: 1) When the navigation angle span is small, simply take the mean value for steering the rover; 2) When the angle span is wide, add some randomness to the steering.
A variable `Rover.rand_angle_thresh` (default = 25000) is defined to determine whether the angle span is narrow or large. Depending on the direction rover is currently driving, a normally distributed extra steering angle with a standard deviation of 3 is added to `Rover.steer`
```python
if len(Rover.nav_angles) >= Rover.rand_angle_thresh:
    if np.random.random() <= 0.4:
       # print("get random direction because angle: ", len(Rover.nav_angles))
        if Rover.steer >= 0:
            Rover.steer = np.clip(np.random.normal(Rover.steer - 5, 5), -15, 15)
        else:
            Rover.steer = np.clip(np.random.normal(Rover.steer + 5, 5), -15, 15)
```

When the rover gets close to the rock.
If rover is already near a rock and in 'stop' mode, then send order to pick it up. However, if Rover is still moving while being near a rock, stop to Rover immediately.
If rover is not too far away from the rock then the rover will brake imediately to lower its speed, prepare to approach the rock in a slow speed. In this step, `Rover_steer` is taken over by `Rover.sample_angle` instead of `Rover.nav_angles`. 

```python
elif Rover.sample_angle is not None and len(Rover.sample_angle) >= 15:
    Rover.steer = np.clip(np.median(Rover.sample_angle * 180/np.pi), -15, 15)
    if Rover.vel >= 1:
    # "STOPPING for TO ROCKS"
    Rover.throttle = 0
    Rover.brake = Rover.brake_set
    Rover.mode = 'stop'
    else:
    # Going for rocks"
    Rover.throttle = Rover.throttle_set
    Rover.brake = 0
    Rover.mode = 'forward'
```



#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

I used 800x600 resolution, "Fantastic" graphic quality, and 22 frames per second in the simulator. Under these settings, I rarely need to swtich to manual mode to get Rover unstuck. In 622s the rover got 89.1% of the world mapped at 73.3% fidelity, rover is also able to identify all 6 rocks and pick up 4 of them.

![alt text][image2]

How I might add improvement:
   1. Add function to deal with stuck issue. Sometimes the rover got stuck after picking up the rock. The rocks are usually close to the nagivable terrain boundary so the rover might have insufficient foward steering angle. If the rover is stuck in forward mode for a certain amount of time, it should switch to back-up a little bit and then go back to forward mode.
   2. Set a potential function for worldmap so that explored map area will have less potential(likelyhood) to be visited again. Then the rover can explore the whole map faster.








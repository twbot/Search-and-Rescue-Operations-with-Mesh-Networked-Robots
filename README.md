# Search and Rescue Operations :: Mesh Networked Robotics

![Grid Image][image1]

[//]: # (References)

[image1]: ./calibration_images/example_grid1.jpg
[image2]: ./calibration_images/angle_example.jpg
[image3]: ./calibration_images/transform_ex.png
[image4]: ./calibration_images/thresholding.png
[image5]: ./calibration_images/coordinate.png
[gif1]: ./output/giphy.gif

---

## Notebook Analysis

Within this [Jupyter Notebook](./code/Rover_Project_Test_Notebook.ipynb) are all the functions providing the necessary data that will allow the rover to autonomously navigate its environment.

#### Sample Data

In the simulator, you can toggle a grid onto the the ground for calibration. The image below is an example. We will use this as a reference for the perspective transform function, allowing us to choose source points, as well as adjust the offset value from the bottom, to account for image data directly in front of the rover.

![Grid Image][image1]

#### Perspective Transform

This function allows us to tranform the image from front point-of-view to a top-down point-of-view, allowing us to calculate the average angle based on available path pixels, once color thresholding is applied, which is the next step.

![Perspect Transform][image3]

#### Color Thresholding

We will apply three seperate color thresholding functions to account for the navigable terrain, obstacles, and the goals. You could condense this into one function, and pass in the rgb threshold, I just prefer this method as it's cleaner and one can know exactly which threshold you're applying via the naming conventions. Although I only use the goal and navigable terrain thresholding for the rover, I plan on coming back and using obstacle thresholding to closely follow the wall, as it is apparently the most efficient navigation method.

Using the RGB image splitting function, I looked at the intensity values and approximated the values that would be needed in order for the robot to "see" its goal points and its obstacles.

The resultant image is an image that only displays the information we want from that particular function. In the case of the goal_thresholding function, it shows a pixel intesity of 1 at every threshold point.

![Threshold][image4]

#### Coordinate Transformation - Rover

In order for the environment to be correctly observed, we "flipped the pixels" to rover-centric coordinates. It first grabs all non-zero pixels from the resultant images of the color thresholding functions, and flipped to seperate axis, while also applying some filters (subtractng to flip rover x-axis, dividing x axis for steering, etc.)

![Coordinate Transform - Rover][image5]

#### Coordinate Transformation - World

We then transform the pixels to world map coordinates via the rotation function, and then a transform, allowing us to update the map real time while we traverse the environment. The obstacles, navigable terrain, and goal points are colored in respectively. Below is an example of the update.

![World Transform][gif1]

---

## Autonomous Navigation and Mapping

Using the functions defined above as tools, we then define two functions that will allow the Rover to percieve and act upon its environment.

#### Perception

In the perception step, we use the prevoiously defined functions to convert the pixels to the rover coordinate frame, the world coordinate frame, and then convert pixels from the respective rover coordinate frame to a polar coordinate frame, allowing use to determine distance of each pixel from the origin, along with the angle from the x-axis. We calculate the average angle and distance for the navigable terrain parameter, along with the goal point parameter. We also add the floored current pixel positions to a discovered-map parameter, which will allow us to create boundaries (which I'll get back to sometime in the future). The last step was to increase the fidelity of the system by limiting the rover to only map pixel coordinates if its pitch and roll angles are within a predefined limit.

#### Decision

In the decision step, we distinguish between navigable terrain angles and goal point angles. If a goal point is located, the systems primary focus will be to navigate toward the goal point, otherwise its business as usual.

##### Parameter Tracking

Within the navigable terrain block, we keep track of several parameters of the rover. Below each script is a description.

___

1. Stuck

```python
if (Rover.throttle > 0) and (Rover.vel < 0.3) and (Rover.forward_time > 3) and not(Rover.picking_up):
            Rover.throttle = 0
            Rover.mode = 'get_unstuck'
```

Forward_time is a parameter that will allow us to determine the amount of time the rover has been trying to move forward. This in conjunction with a velocity smaller than 0.3 should help us to determine if the rover is stuck. The forward_time parameter is incremented within the forward function. Once all conditions are met, it sets the rover's mode to 'get_unstuck'

___

2. Circle

```python
if (Rover.steer == 15) or (Rover.steer == -15):
            if(Rover.steer == Rover.past_steer):
                Rover.steer_time = Rover.steer_time + 0.03
            else:
                Rover.steer_time = 0
            if(Rover.steer_time > Rover.max_steer_time):
                Rover.mode = 'steer_anew'
                Rover.to_steer = Rover.steer
```

Past_steer is a parameter that sets itself to the prior steering angle. If the steering angle is equal to either of the extemes, and this steering angle is equal to the prior, increment steering time. Once the steering time is greater than 10 seconds, it sets the rover's mode to 'steer_anew'

___

3. Sample Collected

```python
if Rover.samples_collected == 6:
            if(np.abs(Rover.pos[0] - Rover.start_pos[0]) < 6 and np.abs(Rover.pos[1] - Rover.start_pos[1]) < 6):
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.finished = True

```

Start_pos is determined in the supplemental_functions.py file, where at the beginning, when total_time equals 0, we set it equal to rovers starting pos. We then track the number of samples the rover has collected, and navigate until the (x,y) position of the rover is within six digits.

##### Modes

When the conditions are met above, the following modes are called. Below each script is a description.

1. Unstuck

```python
elif Rover.mode == 'get_unstuck':
            if (Rover.stuck_time < Rover.max_stuck_time):
                Rover.throttle = -0.2
                Rover.steer = 15
                Rover.stuck_time  = Rover.stuck_time + 0.03
                Rover.mode = 'get_unstuck'
            else:
                #In case rover is stuck in different situation
                if(np.abs(Rover.pos[0] - Rover.last_pos[0]) < 0.3 and np.abs(Rover.pos[1] - Rover.last_pos[1]) < 0.3):
                    Rover.mode = 'maneuver'
                    Rover.forward_time = 0
                    Rover.stuck_time = 0
                #else rover was freed
                else:
                    Rover.brake = Rover.brake_set
                    Rover.throttle = Rover.throttle_set
                    Rover.forward_time = 0
                    Rover.mode = 'forward'
                    Rover.stuck_time = 0

```

We'll track the amount of time maneuvering out of the stuck position with stuck_time. We set the throttle to a negative value to encourage the rover to move backward at a steering degree of 15. These are arbitrary values I picked, and could obviously be changed to increase the fidelity of the system. Once the max time has passed, we check if the rover is still stuck by checking if the last position and the current position are within a set tolerance. If so, it goes onto a different maneuvering mode. If not, we continue forward.

___

2. Maneuver

```python
elif Rover.mode == 'maneuver':
            if(Rover.maneuver_time < 2):
                Rover.throttle = 0
                Rover.steer = 15
                Rover.maneuver_time = Rover.maneuver_time + 0.03
            else:
                Rover.mode = 'forward'
                Rover.maneuver_time = 0

```

Maneuver is called if the rover is unable to become unstuck after "get_unstuck" is called. This just turns the rover without a throttle being applied.

___

3. Steer Anew

```python
elif Rover.mode == 'steer_anew':
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            if(Rover.duration_steer < 2):
                if(Rover.to_steer == 15):
                    Rover.steer = -15
                else:
                    Rover.steer = 15
                Rover.duration_steer = Rover.duration_steer +0.03
            else:
                Rover.mode = 'forward'
                Rover.throttle = Rover.throttle_set
                Rover.duration_steer = 0
                Rover.steer_time = 0
```

Steer_anew is called once the robot has been determined to be steering in a circle for an x amount of time. It changes the steering angle and continues forward.

---

## Conclusions

On average, the rover maps around 94% of its environment at approximately 80% fidelity. Time taken is a metric I could improve upon, and hope to in the future using boundaries based on the discovere_map parameters I have added. I also wish to increase the fidelity of the system.

##### Future Plans

1. Increase fidelity of the system.
2. Use map boundaries.

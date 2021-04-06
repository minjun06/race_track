## Race Track



## Environment Setup

### Dependencies

- github.com/aws-robotics/aws-robomaker-racetrack-world

```
cd ~/catkin_make/src
git clone https://github.com/aws-robotics/aws-robomaker-racetrack-world.git
```



## Launch

`roslaunch race_track racetrack.launch `



## Obstacle Avoidance Challenge Based on Scan Data

by **Tongkai**

**Code Base**: https://github.com/minjun06/race_track/tree/main

**Demo Video Link**:  https://drive.google.com/file/d/1wxwmjiL0VSVjHYOCXwvkuPu0qhCMLytD/view?usp=sharing

### Related State

`FOLLOWING_LINE`: robot racing at 1.5 meters per second

`OBSTACLE_DETECTED`: velocity is decrease to 1 meters, and turning to avoid the obstacle automatically

`WAIT`: when an obstacle is detected in front of the robot, the robot will wait for a fixed amount of time before actually avoid the obstacle. The reason for a wait state is that the obstacle might be moving away in a short period of time, so there is no need to avoid. **In this assignment, wait time is set to zero**

`FOLLOWING_LINE -> OBSTACLE_DETECTED`:  when there is obstacle within 3 meters in its head region

### Turning strategy

![image](./images/image.png)

As is shown in the sketch above, the obstacle avoidance strategy is based on the Lidar data.

We just find a cheaper orientation to turn:

- Head region is 20 degrees  and 3 meters long

- Let the degree for left boundary of the obstacle be `L` , degree for right boundary of the obstacle be`R`.

  When an obstacle is detected, we also get `L` and `R` from `/scan`. 

- Turning left: left_angle = angle(head_right_boundary, `L`)
- Turning right: right_angle = angle(hear_left_boundary, `R`)
- if left_angle > right_angle, turn right, otherwise turn left

### Experiments on Different Obstacle Patterns

These three kinds of obstacles can be avoided. Please refer to the demo video.

- simple one at left side: turn right

  ![image-20210405224301567](./images/image-20210405224301567.png)

- simple one at right side: turn left

  ![image-20210405224429649](./images/image-20210405224429649.png)

- complex pattern: turn left then right to avoid all

  ![image-20210405224600272](./images/image-20210405224600272.png)

-----






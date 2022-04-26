# CMP3103M-autonomous-mobile-robotics-assessment
A ROS script to navigate a maze created for the CMP3103M-2122 Autonomous Mobile Robotics - Assessment 1.

## Summary of solution
This solution will solve a maze using both laser distance measurements and an image stream from a turtle bot. The solution uses the laser distance measurements to avoid walls, and to guide it through a maze, by following a path decided by moving towards the most open areas (direction of the furthest wall). It uses the image stream to avoid red squares, move towards blue squares (helping it towards the goal) and move towards and stop on green squares, where it then finishes.

## To run the solution
1. Firstly make sure you are running the script on the correct system, details of how to do so can be found here: [LCAS Home Installation](https://github.com/LCAS/teaching/wiki/Home-Installation)
2. Secondly, run the turtlebot simulator, for example opening the simulator with maze 1:

```
roslaunch uol_turtlebot_simulator maze1.launch
```

3. Navigate into directory of this solution and run it:

```
python maze-navigator.py
```

# NeatoFollowingNeato

## Run



## Overview
Quite true to its name, this project is about Neatos following Neatos that can create a line of slithering Neatos, with the only limiting factor being the number of available Neatos to connect to. This project was inspired from an in-class lab, "Neato Soccer" (link here).

## Approach 1 - Color Detection

### Demo

[![Demo Video](https://img.youtube.com/vi/cAolaKo4dqg/maxresdefault.jpg)](https://youtu.be/cAolaKo4dqg)

[![Tracking Comparison](https://img.youtube.com/vi/BhG1ZUt1OvM/maxresdefault.jpg)](https://youtu.be/BhG1ZUt1OvM)

### Design Decisions

1. Automatic boundary setting through clicking color
2. Testing mode / Real mode
3. Proportional speed through lidar sensor vs hard-coded (angle, linear)
4. Centroid detection through binary image

### Limitations

1. Need specific color attachment
2. Very sensitive to lighting
3. Weird proportional speed

### Improvements

1. Real-time adjustment to the tracking pixel
2. Use HSV instead of RGB
3. Smarter way to achieve proportional speed

## Approach 2 - Keypoint Matching

### Research

Whatever

## Conclusion

### Challenges
- lidar camera cannot recognize neato in front of it unless the neato had something high on it
- neato could not identify the front neato when there was a difference in lighting

### Improvements
- syncing up the speed of the different robots (e.g. using the odometry of each robot)

### Lessons


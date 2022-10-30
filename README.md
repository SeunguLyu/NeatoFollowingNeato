# NeatoFollowingNeato

## Run

- Clone this repo in your ros2 workspace source folder (ex. ros2_ws/src/)
- After build, run the following code:
```
ros2 run neato_following_neato neato_tracker_color 
```

## Overview

Quite true to its name, this project is about Neatos following Neatos that can create a line of slithering Neatos, with the only limiting factor being the number of available Neatos to connect to (Actually, with more computers, we can have infinite number of Neatos following each other). This project was inspired from an in-class lab, ["Neato Soccer"](https://github.com/comprobo22/class_activities_and_resources/tree/main/neato_soccer)

## Goal



## Approach

### Color Detection

[![Tracking Comparison](https://img.youtube.com/vi/BhG1ZUt1OvM/maxresdefault.jpg)](https://youtu.be/BhG1ZUt1OvM)
↑ Click Image

### Keypoint Matching

## Demo

[![Demo Video](https://img.youtube.com/vi/cAolaKo4dqg/maxresdefault.jpg)](https://youtu.be/cAolaKo4dqg)
↑ Click Image

## Design Decisions

1. Automatic boundary setting through clicking color
2. Testing mode / Real mode
3. Proportional speed through lidar sensor vs hard-coded (angle, linear)
4. Centroid detection through binary image

## Conclusion

### Challenges
- lidar camera cannot recognize neato in front of it unless the neato had something high on it
- neato could not identify the front neato when there was a difference in lighting

### Limitations

1. Need specific color attachment
2. Very sensitive to lighting
3. Weird proportional speed

### Improvements

1. Real-time adjustment to the tracking pixel
2. Use HSV instead of RGB
3. Smarter way to achieve proportional speed
4. Control multiple neatos from one computer

### Lessons


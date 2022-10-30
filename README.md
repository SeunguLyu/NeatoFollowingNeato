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

Do achieve the goal of Neato following Neato, there were several tasks:

1. Neato should be able to locate another Neato that is in front of it.
2. Neato should be able to figure out distance from the other Neato.
3. From the information above, Neato should be able to control its own velocity to follow another Neato.

Main concern was the first task. We decided on using computer vision to locate another Neato, and we considered several different algorithms to see what will work well, and also be done in the time frame of this project.

## Approach

### Color Detection

[![Tracking Comparison](https://img.youtube.com/vi/BhG1ZUt1OvM/maxresdefault.jpg)](https://youtu.be/BhG1ZUt1OvM)
↑ Click Image For Video

Color detection was the easiest algorithm to implement, while it was still pretty strong. The basic idea under this method is that you pick all the pixels from the image that falls into specified boundary. For example, if we want to detect "Orange" color, we should give the boundary of (R: 210-250, G: 100-140, B: 0-40) and most of orange-ish color will fall within the boundary. Above video shows how well color detection work real-time.

The main issue with this method was limitation on color choices and instability to lighting. Because Neato did not really have a distinc color that can be seperated from the background, Neato should be attached with something that has distinct color such as post-its. Also, the color value of pixel changes dramatically under different lighting. This means that settings for the boundaries should be adjusted every time program runs, and algorithm might not work well when the Neato is passing by strong light or shadow of an object. 

Despite all the drawbacks, this was the algorithm we focused on. It was still very reliable as far as we can control the environment, and there are some additional algorithms we can implement to get rid of the limitations. 

### Keypoint Matching

Keypoint matching was another algorithm that we tried, but realized it would not be as simple as color detection. We tried first with the SIFT descriptor, and immediately realized several issues:

1. Neato's shape makes it hard to have many descriptors on the Neato (especially it is far away), making it difficult to match. 
2. Keypoint matching had same issue with lighting as the color detection, where it became harder to recognize Neato as the lighting changed from the given image.
3. As Neato approached another Neato, the angle and size camera see the object changed, causing inconsistency in how descriptors are created.

This was the reason why for keypoint matching, we needed to do something more to achieve the task. One method we considered was create a number plate for the Neatos, which will give a distinct object to detect and enhances the visuals. We ended up not trying this due to the time limitation, but we still saw the possibility implementing keypoint matching for this project.

### Algorithm 3? 

If any, mention here.

## Demo

[![Demo Video](https://img.youtube.com/vi/cAolaKo4dqg/maxresdefault.jpg)](https://youtu.be/cAolaKo4dqg)
↑ Click Image For Video

We decided to achieve the task with real-time color detection through camera and LIDAR scan. Under controlled environment, where there is no big difference in lighting, no extra object that falls into color boundary, the code works well and achieved the goal of following Neato in front of it. Actually, we were able to run the code in multiple Neatos and created something like a Neato-train!

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


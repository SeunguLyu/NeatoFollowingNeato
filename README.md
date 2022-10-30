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

For the second task, this can be done through two different ways. Depending on the algorithm, the computer vision part can figure out the distance between the Neatos. Also, Neato has built-in LIDAR scan that can detect objects around it, so as far as we know the angle between two Neatos we will also be able to get distance between them.

For the last task, it will be best to implement proportional velocity control based on the angle/distance we got from step 1 and 2. 

## Approach

### Color Detection

Color detection was the easiest algorithm to implement, while it was still pretty strong. The basic idea under this method is that you pick all the pixels from the image that falls into specified boundary. For example, if we want to detect "Orange" color, we should give the boundary of (R: 210-250, G: 100-140, B: 0-40) and most of orange-ish color will fall within the boundary. Despite the simplicity, color detection worked well real-time.

The main issue with this method was limitation on color choices and instability to lighting. Because Neato did not really have a distinc color that can be seperated from the background, Neato should be attached with something that has distinct color such as post-its. Also, the color value of pixel changes dramatically under different lighting. This means that settings for the boundaries should be adjusted every time program runs, and algorithm might not work well when the Neato is passing by strong light or shadow of an object. 

Despite all the drawbacks, this was the algorithm we focused on. It was still very reliable as far as we can control the environment, and there are some additional algorithms we can implement to get rid of the limitations. 

### Keypoint Matching

Keypoint matching was another algorithm that we tried, but realized it would not be as simple as color detection. We tried first with the SIFT descriptor, and immediately realized several issues:

1. Neato's shape makes it hard to have many descriptors on the Neato (especially it is far away), making it difficult to match. 
2. Keypoint matching had same issue with lighting as the color detection, where it became harder to recognize Neato as the lighting changed from the given image.
3. As Neato approached another Neato, the angle and size camera see the object changed, causing inconsistency in how descriptors are created.

Because of these reasons we needed to do something more to achieve the task through keypoint matching. One method we considered was create a number plate for the Neatos, which will give a distinct object to detect and enhances the visuals. We ended up not trying this due to the time limitation, but we still saw the possibility implementing keypoint matching for this project.

### Algorithm 3? 

If any, mention here.

## Demo

[![Demo Video](https://img.youtube.com/vi/cAolaKo4dqg/maxresdefault.jpg)](https://youtu.be/cAolaKo4dqg)
↑ Click to view video

We decided to achieve the task with real-time color detection through camera and LIDAR scan. Under controlled environment where there is no big difference in lighting, and no extra object that falls into color boundary, the code works well and achieved the goal of following Neato in front of it. Actually, we were able to run the code in multiple Neatos and created something like a Neato-train!

## Design Decisions

[![Tracking Comparison](https://img.youtube.com/vi/BhG1ZUt1OvM/maxresdefault.jpg)](https://youtu.be/BhG1ZUt1OvM)
↑ Click to view video

### Attaching Post-its to Neato

![](images/post-its.png)

We found immediately that Neato did not have a distinct color that we can use for the color detection. The decoration that covers LIDAR sensor had color that we can detect (such as blue, green, purple) but they were too small and hard to detect in longer distances. Also, we realized that Neato's LIDAR scan cannot figure out where the other Neato is because the LIDAR sensor is located above Neato's height. To solve both of these problems, we attached post-its with different colors - this will provide Neato with a color that is different from the environment, and also raise Neato's height so that it shows up on the LIDAR scan. 

### Automatic Boundary Setting

![](images/auto_boundary_short.gif)

As we mentioned earlier, color value of pixels change dramatically under different environments. So there was need to set up the color boundaries automatically to adjust to new environment, and this was done by clicking a pixel from the camera feed. For example, if we set boundary range to 30 and click a pixel that is (R: 150, G: 120, B: 90) then the boundary will be (R:120-180, G: 90-150, B: 60-120) and will detect every pixel in that range. 

### Centroid Detection Through Binary Image

![](images/centroid_example.png)

Once we set up the boundaries and create a binary image, we can calculate centroid of all the pixels in the boundary. To do this, we used OpenCV's moments function.

```python
moments = cv2.moments(self.binary_image)
if moments['m00'] != 0:
    self.center_x = int(moments['m10']/moments['m00'])
    self.center_y = int(moments['m01']/moments['m00'])
```
Through this process, we were able to get the position of centroid on the current frame. You can see on the left part of the picture that shows centroid as green circle. Once we know the circle, we can calculate the angle between two Neatos, information that we can use to rotate the Neato toward the other Neato's direction. Getting the equation to get exact angle value was done through manual calibration and measurments, since each Neato is set-up differently.

### Proportional Angular and Linear Velocity

![](images/proportional_speed.png)

```python
self.drive_msg.angular.z = -self.tendency_x * 2
if self.isProportional:
    if self.lidar != None:
        angle = int(-self.tendency_x * 100)
        distance = self.lidar.ranges[angle]
        # ignore occasional 0 value for the distance
        if distance < 0.5 and not distance == 0.0:
            self.drive_msg.linear.x = 0.1
        elif not distance == 0.0:
            self.drive_msg.linear.x = (distance-0.5) * 0.2 + 0.1
```

From the angle information, self.tendency_x (angles from -50 to 50 degrees, but divided by 100), we can set the angular speed to be proportional. Neato will rotate faster if there is big angle difference, slower if tendency_x is closer to 0. 

Proportional linear speed is bit more complicated. We had to use the Neato's LIDAR scan data, and since we know the angle between two robots, we just had to look at the single scan data at the angle. From there we can get the distance between two robots, which can be used to control how fast the Neato should approach another Neato. 

### Testing Mode and Real Mode

One last design choice we made was implementing the test mode. During test mode, we can check our color tracking algorithm using pre-recorded videos, which helped a lot in fast prototyping. The code is written so that transition between real mode and test mode is simple, just setting a boolean value to True or False. Next plan was to improve test mode so that it can also work with bag files, which would give more realistic simulation.

## Challenges

### Modification to Neato Required

It was challenging to make the project work without doing any modification to Neatos. For the particular implementation we did, we needed to attach 4~5 post-its to the Neatos for color detection and LIDAR scan. This raises the complexity as we add more and more Neatos to the algorithm - for example, in the demo video we presented in the beginning, we had to choose two colors that are very different to avoid possible boundary overlap. I guess maximum number of Neatos we can control at the same time would be around 4~5 due to this limitation. Also, setting up takes extra time, which was a big factor that slowed down our testing process. 

### Difference in 
3. Weird proportional speed

## Improvements

1. Real-time adjustment to the tracking pixel
2. Use HSV instead of RGB
3. Smarter way to achieve proportional speed
4. Control multiple neatos from one computer

## Lessons


[//]: # (Image References)
[image_0]: ./misc/rover_image.jpg
[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)
# Search and Sample Return Project


![alt text][image_0] 

This project is modeled after the [NASA sample return challenge](https://www.nasa.gov/directorates/spacetech/centennial_challenges/sample_return_robot/index.html) and it will give you first hand experience with the three essential elements of robotics, which are perception, decision making and actuation.  You will carry out this project in a simulator environment built with the Unity game engine.  

## Obstacle and Rock Sample Identification
To identify obstacles, the color\_thresh function in perception.py was modified to consider all pixels that do not meet the color threshold as obstacles  (lines 14 and 23) and return that map to the perception\_step method.
To identify rock samples, a rock\_extractor method was defined in perception.py that color thresholds the color yellow and returns that locations of such pixels (lines 27-33) similar to the technique used [here] (http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html). An example on how the rock\_extarctor method was used is shown in code-cell 12 in the jupyter notebook.


## process_image() method
This method is responsible for to analyze images and create a worldmap. The method is implemented in code-cell 9 in the jupyter notebook.
The first step was to determine the source and destination points to be apply perspective transform. The grid in the simulator was used to determine the source points, and the destination points represent a 10x10 pixel square.
After the perspective transform is applied, the color\_thresh and rock\_extractor methods are used to determine the locations of navigable terrain, obstacles, and rock samples.
Afterwards, the thresholded images are converted to robot-concentric coordinates, which are then rotated and translated to map the pixels in world coordinates.
A video in th output folder shows how the process_image() analyzes the images and creates the worldmap.

## perception_step() and decision_step() methods
The perception_step() method in perception.py is an enhanced version of the process\_image() method defined in the jupyter notebook. The perception\_step() method does the same functionalities as process\_image() in addition to filtering images that are valid for mapping based on setting thresholds near zero in roll and pitch. It also calculates the navigable distances and angles to feed the decision\_step() to help the rover move. Moreover, it calculates the distance and angle of detected rock samples to feed them to the decision\_step() and help the rover collect the rock sample.
The decision\_step() in decision.py is the brain of the rover. It includes a few conditions to check the current state of the rover, and take actions accordingly. The code is commented to explain the different conditions and the rationale behind each step.


## <span style="color:red">NOTE</span>
The collect\_rocks state parameter in drive_rover.py is set to True by default. This allows the rover to collect samples if they are located. The collection algorithm fails in some cases, like when two samples are located in the same image. In order to disable rock sample collection and have a code that meets the fidelity and mapped percentage requirements, one can set the collect\_rocks parameter to False.


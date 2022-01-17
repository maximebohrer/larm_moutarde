LARM - Moutarde - Channenge 2
=============================

The goal of the challenge is to demonstrate the capability the robot has to map an environment and to retrieve specific objects on it.
 
/!\ The project depends on `mb6-tbot`


Detection in the image
----------------------

To detect the bottles in an image from the camera, we trained a cascade classifier.

Our model has been specically trained to recognize black bottles of Nuka Cola®. We used a cascade classifier with Local Bynary Patterns (LBP). We also generated our own large data base using the camera of the robot (intel realsense D435) to obtain images as close as possible to the images that will be seen in the rosbag or by the robot (The script we used to generate these images can be seen in `training.py`).


Processing the data from the camera
-----------------------------------

First, the distance from the camera to the bottle is calculated using the depth image. Then, the 3D coordinates of central point of the bottle in the camera frame is calculated using ROS' image_geometry library. Finally, the coordinates are converted in the map frame using the transformations published on tf.

The first verification that we do to filter out false alarms is to check if the size of the bottle is coherent.

This process is explained in more details in the python script `bottle_detector.py`


Keeping track of the bottles
----------------------------

The `Bottle` class contains all the code used to keep track of the position of the bottles.

For each frame, the main class passes the list of points it found to the Bottle class. For each point, if a bottle that is very close already exists, it is updated: its coordinates are adjusted. Otherwise, it is created. A newly created bottle has to be detected and updated enough times before beeing published. It is then marked as listed. Similarily, if a bottle that is not listed is not detected for too long, it is considered as a false alarm and is destroyed, without having been published.

This process is explained in more details in the python script `bottle_detector.py`


Optionnal Features
------------------

1. Rviz

Information is returned to rviz thanks to the `challenge2.launch` which launches rviz with the configuration saved in the file `config_ch2.rviz`. The map, the laser, the bottle markers and an image containing the detected bottles are shown.

2. Shape of map 

The map is fully handled by the gmapping node.

3. 2d-version-bottle

We trained our model with black bottles, so it only detects the black bottles. See the "Detection in the image" paragraph.

4. Position of bottle

We have used several technique in order to increase the precision of the bottle's position. First, we used the image_geometry library to translate the postion of the bottle in the image (in pixels) to the 3D plan of the camera, rather than doing the trigonometry calculations by ourselves. We also saved the time stamp of the image to give it back when doing the transformations and placing the marker. This way, the transformations are calculated with the tf tree of the instant where the image was taken, and there is no ambiguity, even if there is some latency due to computiong time.

5. The position of the bottle is streamed one and only one time

See the "Keeping track of the bottles" paragraph. Each time a bottle is detected, its position is adjusted, by taking the average of the last few positions. By default, the marker is updated so that rviz can display it at the correct place: it is re-published with the SAME ID, so it is not new marker. If you want each marker to be published only one time, you can change the `publish_updates` variable to `False` in the `bottle_detector.py` script, line 106.

6. All the bottles are detected

Our model works pretty well on standing bottles. Unfortunately, the second model that we tried to train for lying bottles still has trouble seeing them, but we will work on that for the challenge 3.

7. Only the bottle are detected

Thanks to the various filters (see the "Processing the data from the camera" paragraph and the "Keeping track of the bottles" paragraph), we've had very few false alarms in the test rosbag.

8. Service

In a terminal, type:
```bash
rosservice call /print_bottles
```
You shold get an answer like
```
success: True
message: "3 bottle(s) [2.001 -0.349 0.105] [-0.09 1.545 0.114] [-1.354 -0.849 0.128] "
```

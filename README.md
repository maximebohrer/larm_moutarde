LARM - Moutarde - Channenge 3
=============================

The goal of the challenge is to demonstrate the capability the robot has to map an environment, to navigate it autonomously, and to retrieve specific objects in it.
 

à dire : grande tolérence mais si 2 bouteilles dans meme image etc
la laser est trop haut pour les bouteilles dont on envoie dans un topic


Dependencies
------------

The project depends on [`mb6-tbot`](https://bitbucket.org/imt-mobisyst/mb6-tbot/)

The project depends on [`move_base`](http://wiki.ros.org/move_base) and [`dwa_local_planner`](http://wiki.ros.org/dwa_local_planner)
```bash
sudo apt install ros-noetic-move-base
sudo apt install ros-noetic-dwa-local-planner
```

The project depends on [`explore_lite`](http://wiki.ros.org/explore_lite)
```bash
sudo apt install ros-noetic-explore-lite
```

Autonomous navigation
---------------------

We decided to use `move_base` from ROS' navigation stack in order for the robot to move autonomously in the arena. We also used the `dwa_local_planner` to replace move_base's default local planner as it works way better in our experience. We configured the global planner so that you can make the robot explore unknown areas, and it will adjust its path along the way as it detects new walls and obstacles. That way, you can make the robot explore its environment easily by sending it goals through Rviz, even if it does not know the map yet.

`explore_lite` is a ROS package that works with move_base. It determines the best goal to go explore in order to map the environment in the most efficient way possible. The goals are sent to move_base and the robot keeps exploring until the whole environment has been mapped. As you will see, it works really well in the simulation, but we had trouble making it work in small cluttered places like the arena. That is why we only enabled it for the simulation.


Detection of the bottles in the image
-------------------------------------

To detect the bottles in an image from the camera, we trained a Haar cascade classifier.

Our model has been specically trained to recognize black bottles of Nuka Cola®. We used a Haar cascade classifier with Local Bynary Patterns (LBP). We trained the model using our own large data base containing about 200 positive images and 400 negative images that we took directly using the camera of the robot (intel realsense D435) to obtain images as close as possible to the images that will be seen by the robot (The script we used to generate these images can be seen in `training.py`).


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
You should get an answer like
```
success: True
message: "3 bottle(s) [2.001 -0.349 0.105] [-0.09 1.545 0.114] [-1.354 -0.849 0.128] "
```

Optionnal Features
------------------

1. The robot detect 2d-version-bottle (black one).

We trained our model with black bottles, so it only detects the black bottles. See the "Detection in the image" paragraph.

2. There is no need to publish goal positions. The robot is autonomous to achieve its mission.

This works in the simulation, but we still have to publish goal positions when using the real robot.

Any suggestions provided by the group are welcome.
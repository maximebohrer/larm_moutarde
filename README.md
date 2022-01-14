# larm_moutarde

Branche pour le challenge 2 : The goal of the challenge is to demonstrate the capability the robot has to map an environment and to retrieve specific objects on it.
 
 /!\ Le projet nécessite les paquets de mb6-tbot

Optionnal Features :
    1 . Rviz
    information is returned to rviz thanks to the challenge2.launch wich launch rviz with the configuration saved in the file config_ch2.rviz
    
    2 . shape of map 
    The map is fully handled by the gmapping node
    
    3 . 2d-version-bottle
    Our model has been specically trained to recognize black bottle of Nuka Cola®. We used a cascade classifier with Local Bynary Patterns (LBP). In order to improve the efficiency of the classifier, we trained one to recognize upright bottles and one for lying ones because of the signicative change of the shape of the object when the bottle is lying on the floor. We also generated our data base using the camera of the robot (intel realsense D435i) to obtain images as close as possible of the images that will be seen in the rosbag or by the robot when in action.
    
    4 . Position of bottle
    We have used several technique in order to increase the precision of the bottle's position. First, we used the image_geometry library to translate the postion of the bottle in the image (in pixels) to the 3D plan of the camera. We also saved the time stamp of the image to give it back placing the marker. It permit the tf to do the transformation with the tf tree of the instant where the image was taken in order to compensate some latency between when the image is taken and when the marker is placed. The latency may appear with computing time.
    
    5 . No redundancy
    It hasn't been dealt with yet.
    
    6 & 7 . All and only all bottle 
    See the point 3
    
    8 . Service
    It hasn't been dealt with yet. 

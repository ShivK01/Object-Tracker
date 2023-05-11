# Object-Tracker
This repository is for a program based on the FTC platform and control hub. It controls a development robot whose primary goal is to self center on a yellow object. Additonally, the robot has a turret and therefore, the camera is able to rotate to center on a yellow object while keeping the robot stationary.

The code is divided into different subsystems. The mecanum drive is controlled through PID and is able to turn to a certain headin using the gyro sensor, or is able to drive a given distance using odometry pods, or can drive for a certain time using a coded timer. The turret is motor controlled as well and is able to rotate to a given heading or simply rotate clockwise/counterclockwise as needed. The camera is mounted to the turret and uses the pipeline to filter yellow objects and center on them.

I chose yellow randomely however other colours can be used as well (as explained in code comments). The camera pipeline uses colour masks of a general and then specific threshold to filter out any non-yellow objects from view. Thus, the outputted camera view is all black except for any yellow object which will clearly show up. This allows the robot to center itself or center the turret on such object by analyzing the position of the object in the camera frame. The advantage of masking out the background is to prevent it from intefering in the location analysis and additonally, adding masks helps accuracy and consistency in different light conditions.

# Operation Modes
The auto mode drives the robot forward 130cm before turning it clockwise 90deg and having the robot center on a yellow pole. This mode was to test the autonomous drive mode based on a set-up I had crerated. 
The CameraMove mode is to test the movement of the camera in response to a moving yellow object.
The TeleOpDrive is to manually control the drive and turret rotation.

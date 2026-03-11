# Terrain-following-UAV-using-Depth-Camera-feed
 Objective: To achieve terrain following by utilizing the drone’s depth camera feed


## Overview of algorithm
The drone is equipped with two RealSense depth cameras in this simulation.

Downward-facing camera: Processing is straightforward — we identify the tallest tree and its location by finding the pixel with the smallest depth value in the image.

Front-facing camera: We generate a terrain map using an edge detection algorithm. This visualization can be viewed by running front.py. For a more optimized approach that only searches for the first relevant pixel, f2.py can be used.



Once we have data from front and downward facing camera alt_maintain11.py performs decision making for which feed to prioritise and adjusts the z velocity of drone accordingly

## Using Intel real sense hardware

First build [Realsense SDK](https://www.mouser.com/applications/getting-started-with-realsense-d455/)
Then run following command to initialise its node

    roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
    


Now the f2.py equivalent for real hardware is r_front.py

## Note:
The data processing scripts also subscribe to imu data and adjust the data according to the drone tilt to simulate the data in reference to ground plane

# Roll, yaw, pitch in manual control

In altmaintain10.py, the joystick is bridged to ArduPilot’s Guided mode, allowing the pilot to control the drone’s roll, yaw, and pitch, while altitude is maintained autonomously.



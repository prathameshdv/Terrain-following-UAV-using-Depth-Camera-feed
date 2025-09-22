# Terrain-following-using-Depth-Camera-feed
### Objective: Terrain following using depth camera feed
## This project assumes that you are working on Ubuntu 20.04

# Build required SDK's
[Ardupilot](https://docs.google.com/document/d/1ihAxgX1y3yRMqRnX1yWfk9WDaxZt8JmtFyEddi13SWw/edit)

[Realsense SDK](https://www.mouser.com/applications/getting-started-with-realsense-d455/)




    
[Screencast from 08-07-24 03:21:10 PM IST.webm](pth.mp4)


# Overview of algorithm
The drone has 2 realsense depth cameras attached to it in this simulation

Processing downward depth data is fairly easy we are looking for the hieghest tree and its position by searching for the lowest depth pixel value and its position in image

For front facing camera we are forming a terrain map by using a edge detection algorithm, this visualisation can be done by running front.py, f2.py is more optimised version of it just looking for first such pixel
![Screenshot from Screencast from 17-06-24 03:25:06 PM IST webm](dep.png)


[Screencast from 08-07-24 03:21:10 PM IST.webm](pth.mp4)

Once we have data from front and downward facing camera alt_maintain11.py performs decision making for which feed to prioritise and adjusts the z velocity of drone accordingly

# Using Intel real sense hardware

First build [Realsense SDK](https://www.mouser.com/applications/getting-started-with-realsense-d455/)
Then run following command to initialise its node

    roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
    
Now you can visualise depth data using  rviz or d2.py

![Screenshot from 2024-06-28 14-15-33](https://github.com/codemaster1104/UAV/assets/115527374/1f188de1-9f4e-40df-9005-3d6f2650f6e8)

Now the f2.py equivalent for real hardware is r_front.py

## Note:
The data processing scripts also subscribe to imu data and adjust the data according to the drone tilt to simulate the data in reference to ground plane

# Roll, yaw, pitch in manual control

In altmaintain10.py joystick has been bridged to the guided mode of ardupilot so that pilot can control roll, yaw and pitch of drone whereas altitude is taken care of autonomously

[▶️ Watch Demo Video](https://github.com/prathameshdv/Terrain-following-using-Depth-Camera-feed/blob/main/pth.mp4?raw=true)


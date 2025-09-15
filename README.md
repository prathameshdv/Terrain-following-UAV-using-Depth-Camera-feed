# Terrain-following-using-Depth-Camera-feed

# UAV project
### Objective: Terrain following using depth camera feed
## This project assumes that you are working on Ubuntu 20.04

# Build required SDK's
[Ardupilot](https://docs.google.com/document/d/1ihAxgX1y3yRMqRnX1yWfk9WDaxZt8JmtFyEddi13SWw/edit)

[Realsense SDK](https://www.mouser.com/applications/getting-started-with-realsense-d455/)

# Tutorial of current algorithms
+ Clone this repo in src folder of catkin_ws
+ get [runway4.world](https://drive.google.com/file/d/1wEHAJMRoC_oQrESQtgkyWg47hVZCOi0y/view?usp=sharing) and add it to your worlds folder in iq_sim
+ change [runway.launch](https://drive.google.com/file/d/1GZMbl0JDaDLkVXpZpK9obQmARWhDPD52/view?usp=sharing) file in launch folder in iq_sim to run runway4.world
+ Open a terminal and run the following scripts parallelly   

  

      cd catkin_ws/
      cd src/
      cd iq_sim/
      roslaunch iq_sim runway.launch
      roslaunch iq_sim apm.launch
        
    This will start the Gzebo world and initialize the ros nodes.
    Now run the startsitl.sh file in **new terminal**

		cd ~
        ./startsitl.sh
Now run the python scripts in src **each in new terminal**

    python ros_script.py
    python ros_front.py
    python f2.py
    python down.py


Now in SITL set the mode to guided and takeoff

    mode guided
    arm throttle
    takeoff 10

Now run the altmaintain11.py script in src

    python altmaintain11.py


    
[Screencast from 08-07-24 03:21:10 PM IST.webm](https://github.com/codemaster1104/UAV/assets/115527374/7340066e-d0f3-4b90-83c0-3dbc77585eac)


# Overview of algorithm
The drone has 2 realsense depth cameras attached to it in this simulation

Processing downward depth data is fairly easy we are looking for the hieghest tree and its position by searching for the lowest depth pixel value and its position in image

For front facing camera we are forming a terrain map by using a edge detection algorithm, this visualisation can be done by running front.py, f2.py is more optimised version of it just looking for first such pixel
![Screenshot from Screencast from 17-06-24 03:25:06 PM IST webm](https://github.com/codemaster1104/UAV/assets/115527374/cb831b4e-7631-4bda-91c0-c376d4c5a3de)

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






https://github.com/codemaster1104/UAV/assets/115527374/d0472412-7819-4752-9598-d71506a3a10b

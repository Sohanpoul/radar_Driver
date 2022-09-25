# radar_driver
    ROS driver for V-MD3 RADAR

# Requirements
    Software :
    1. ROS -1
    2. Socket library
    
    Hardware:
    1. V-MD3 RADAR sensors
    2. Network Switch (based on number of sensors)


# Installation Procedure
    1. Download the package in to catkin workspace 'src' folder.

    2. Set TCP IPv4 address of RADAR sensors in the subnet range 192.168.100.xx using V-MD3 control pannel. Then update the 'TCP_IP' parameter in radar_launch.launch file
    Example - if two radar sensors set IP address of first radar 192.168.100.201 and second radar 192.168.100.203

    3. Set UDP port number of each sensors to different values on python script. Then update the 'port' parameter in radar_launch.launch file
    Example - if two radar sensors set port number of first radar 4567 and second radar 4660

    3. To launch the driver first build the package (if not done before) and then source the setup file. Execute the command:
        roslaunch radar_driver radar_launch.launch

# Output
    Radar data published over ROS can be viewed on the topics /radar_1 and /radar_2
    RADAR data format is:
        1. target_id
        2. distance
        3. speed
        4. azimuth
        5. elevation
        6. magnitude
    More information about these data can be found on V-MD3 datasheet.


Contact Details: sohanpookolayil@gmail.com





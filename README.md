# Line Following Warehouse Robot
Processes images from a Raspberry Pi camera mounted to a Ubiquity Magni robot to navigate around a warehouse to deliver supplies. Written in Python and Robot Operating System (ROS).

## Usage
The Magni robot has four states that it can be in while running automatically. The "home" state, where it waits for a barcode to be scanned (Scanning), a line-following state (Following), a state where it will wait at a designated stop (Halted), and a state to move past stations (Advancing). The robot can also be controlled manually using the attached controller, which pause the line following algorithm.

### Barcodes
While the robot is stationed at the home station, barcodes can be held in front of the front-mounted camera to instruct  Barcodes can encoded in either Code39 or Code128. The encoded text should be in the format of `L=#:S=#,#,#` where L denotes the total number of stations in the loop (not including the home station) and S denotes the stations to stop at. There is a tool to create these barcodes, but can also be created online using this this template.

### Line Following
The robot will follow a green tape line. Tuning values can be edited by connecting to the robot through the network it provides. These configuration files are located in `~/catkin_ws/src/line_follower/config`. To connect to the robot, use the [manufacturer's guide](https://learn.ubiquityrobotics.com/noetic_quick_start_connecting).

There are two factors that will be considered in the line following algorithm - the angle the line is positioned at relative to the robot, and the distance the robot is from the center of the line. If the robot strays too far from the the line, it will correct and recenter itself. The weight of this centering compared to the line angle can be adjusted in the configuration files.

### Waiting at a Stop
When the robot reaches a station, which is denoted by a gap in the tape line the robot follows, it will either advance through the station, or wait at a designated stop. At a stop, the robot can be advanced by holding a hand up to the left-side sonar.

### Advancing Through Stations
When reaching a station that is not a stop, or when the robot is signalled to move past a stop, then the robot will move directly forward until it finds another tape line to lock to.

### Manual Control
To override the automatic algorithm and use the controller, hold the left trigger on the controller. This will halt the robot, and it can then be controlled with the left joystick. When the left trigger is released, the robot will return to the most recent state it was in before being overridden. The connection of the controller can be inconsistent, but unplugging and replugging the dongle back in will usually work.

## The Robot Hardware
### Display
The oled display mounted to the main board of the robot will display important information about the robot as follows:

```
┌──────────────────────────────┐
|             [IP]             |
|                              |
| ST: [Current state]          |
| CD: [Last scanned barcode]   |
| AL: [Angle to line]          |
|                              |
| Bat: [Battery info]          |
| Mot:[Motor power] FW: [Ver.] |
└──────────────────────────────┘
```

### Camera
The camera is a Raspberry Pi Camera Module 2. It is mounted to the front of the robot using a 3D printed bracket (bracket_rev3.STL). In the current setup, it records at 640x480 resolution and 30 fps. This can be changed in the `~/home/ubuntu/catkin_ws/src/magni_robot/magni_bringup/core_launch.em` file.

### Sonar
This robot uses the top-mounted sonars to stop the robot in case something blocks its way when it is following a line. The right sonar does not work, but it is currently unused. Internally, they are labeled 0-4, where right right sonar is 0 and the left is 4. 

## Debugging 
If the robot is not functioning properly, there are multiple ways to find and fix the problem. 

### Line Detection Debugging Image
By running the line_follower package's debugging_image node, a new ROS topic will be published, at `debugging/threshold_image`. This can be accessed by using `rosrun rviz rviz` on a linux workstation on the robot's network and adding a new display for this topic. The line detection algorithm can be sensitive to lighting and the color of the floor beneath the line, so this can help adjusting the configuration values for the camera node.

### Internal ROS Topics
Connecting to the robot will allow some internal topics to be shown using `rostopic echo`. Some helpful topics are:
```
/line_follower/camera_angle  -  same as shown on the robot's display, but at a higher frequency
/line_follower/code          -  same as shown on the robot's display, but at a higher frequency
/line_follower/state         -  same as shown on the robot's display, but at a higher frequency
/cmd_vel                     -  shows the movement commands given to the motor control node
/pi_sonar/sonar_0            -  displays info about the right sonar 
/pi_sonar/sonar_1            -  displays info about the diagonal left sonar 
/pi_sonar/sonar_2            -  displays info about the diagonal right sonar 
/pi_sonar/sonar_3            -  displays info about the forward sonar 
/pi_sonar/sonar_4            -  displays info about the left sonar 
/rosout                      -  shows messages and errors given by the ROS nodes
/joy                         -  displays controller input
```

## Tools
### Barcode Creator App
Included is a GUI to help create and save barcodes as images to the computer. There are three main windows that this application has: the default window, batch creation, and a graphical interface. The default window simply asks for the number of stations in the loop, and the stations to stop at. This window has a preview for the barcode generated. The batch creation window allows for multiple different codes to be generated and saved at once, but does not show previews. Lastly, the graphical interface gives a more straightforward and visual way to select which stations to stop at. The stations are separated into a maximum of 5 groups, the details of which can be edited through a settings menu. By clicking on each group, stations can be selected. Clicking the "Add" button will add the selected information to either the default or batch creation window.

### Barcode Creator Python Script
This script generates barcodes for the magni robot to scan. It is made for use in a command line interface.

### Camera Bracket
There is both a STL and a Solidworks file for the bracket used to secure the camera to the body of the magni robot. It can be 3D printed and secured to the front holes of the frame using 4 bolts and nuts.
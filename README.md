****************************************************
**************       INTRODUCTION      *************
****************************************************

This repository contains a ROS package that runs a demo for hubo through
MAESTRO. The demo receives messages from Hubo's Biotac Sensors via a Biotac
ros node. Depending on the pressure felt by Hubo's right index finger Biotac
sensor, his arm will push outward, stay in place, or retract, allowing Hubo
to interact with anyone by the touch of a finger.

****************************************************
**************       REQUIREMENTS      *************
****************************************************

- Ubuntu 12.04 LTS
- ROS Fuerte
- MAESTRO (ROS stack)
- Penn Haptics Biotac ROS Stack (https://github.com/IanTheEngineer/Penn-haptics-bolt)

****************************************************
**************       INSTALLATION      *************
****************************************************
Once all dependencies have been installed, navigate to the install directory of this
package and run ./install.sh to add this package to your ROS path

After completing this, build this package and all dependencies with rosmake.

****************************************************
*******************    RUN    **********************
****************************************************

This demo needs Maestro to be running to move HUBO. Follow the instructions in
MAESTRO's readme for starting maestro.
To start the Biotac push demo, run the following roslaunch command:

roslaunch BioTacPushDemo biotac_push.launch

****************************************************
*******************    STOP   **********************
****************************************************

To stop the demo, simply press Ctrl+C in the terminal running the demo.

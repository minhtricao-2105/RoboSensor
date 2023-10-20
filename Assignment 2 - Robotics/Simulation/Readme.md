# MARKETMATE FETCHER
## Back ground:
In nowadays trend, the integration of collaborative robots (cobots) is reshaping the dynamics of retail operations. This project explores the innovative application of cobots in the supermarket domain, focusing on their ability to streamline product restocking. By using robot's application which seeks to revolutionize the supermarket experience, optimizing efficiency and enhancing customer satisfaction in the realm of automated retail solutions.

In the project, we decided to use Omron robots which are TM5 (with mobile base) and TM12 for restocking process.
The TM5 will be on top of mobile base which increase the accessibity and flexibity, TM5 also has compact design which make it ideal for filling products in small shelves
TM12 has larger workspace so it can reach items from greater distance and it has better load distribution.

Our control mechanism relies primarily on the robust RMRC (Resolved Motion Rate Control) algorithm, ensuring seamless and precise robot navigation between designated positions within the workspace. Emphasizing user safety, our system incorporates a range of protective measures, including simulated and physical emergency-stop (e-stop) buttons, strategically positioned light curtains, and advanced sonar sensors for collision detection in the workspace. Additionally, a fail-safe collision avoidance feature has been integrated combining with algorithm to check self collision and ground collision, ensuring continued safety even in the event of sensor malfunction, thereby prioritizing a secure operational environment.

Simulated within the Matlab environment, our project envisions a dynamic workspace inspired by the layout of a contemporary convenience store. The process involves the TM12, facilitating the transfer of products to a designated location. Subsequently, the TM5 mobile unit approaches the same point, efficiently picking and storing the items within its container. Following this, the TM5 navigates each shelf, ensuring product placement in their respective locations. During process, system can be able to recover/resume after an e-stop event. 

We also created the GUI for user to interact with the system. The GUI has advanced “teach” functionality that allows jogging the robot. It includes both individual joint movements  plus enable [x,y,z] Cartesian movements. A joystick is can be used as an additional control method beside GUI buttons presses.

During a live robot demonstration, we employed a UR3 robot to execute a pick and place task. Due to the unavailability of two robots simultaneously, we devised an efficient workflow. Initially, we positioned six dice (comprising two blue, two red, and two yellow ones) on the table. To facilitate the process, we integrated an RGB camera to accurately detect the color of each dice, subsequently utilizing image plane coordinates to compute the translation of each dice in the camera's frame. Utilizing the acquired data, we transformed the target points back to the robot's end effector, enabling us to utilize the RMRC (Robot Motion Planning and Control) technique to generate a path from the current pose to the desired pose. In the final stage, leveraging the color information detected by the camera, the UR3 robot efficiently move the dice to pre-defined locations corresponding to their respective colors.

## Contributors:
The contributors in this project are:
  1.  Minh Tri Cao - Student ID Number: 14231702:
    *  Creating world file.
    *  Creating robot model.
    *  Contributing and reviewing mission.
    *  Making GUI.
    *  Testing and verifying code.
      
  2.  Anh Tung Nguyen - Student ID Number: 14072286:
    *  Creating world file.
    *  Creating robot model.
    *  Implement mission.
    *  Reviewing GUI.
    *  Testing and verifying code.

## Requirement:
### Software System:
*  Matlab 2023a
*  Peter Corke Robotics toolbox
*  Ubuntu 20.04
*  Ros Noetic
*  Python3 + OpenCV version 4.0+
### Hardware System:
*  UR3 robot
*  Camera Realsense D435

## Installation and Setup Guide
### Simulation Environment
  1. Prerequisites:
  Before diving into the project setup, ensure you have the following prerequisites installed:
     *  Matlab: Our project heavily relies on Matlab functionalities, we are currently using Matlab 2023a version.
     *  Peter Corke's Modified Robotics Toolbox: This should be provided by UTS teaching staff.
     *  Following Matlab's toolbox: Simulink, Simulink Real-Time, Simulink 3D Animation, Optimization Toolbox, Robotics System Toolbox, ROS Toolbox, Statistics and Machine Learning Toolbox, and the Symbolic Math Toolbox.
  Note: Detailed installation instructions for these tools can be found on their respective official websites.

  2. Running project:
     *  Run the file named 'MainGui.mlapp'
     *  CLick 'Load Simulate' to launch the environment
     *  Click 'Run Mission' to start
     *  Click 'E-Stop' to pause the process. Then toggle 'E-Stop' switch and click 'Resume' to continue
     *  Click 'Cancel' to cancel mission
  If you want to use Teach Pedant
     *  Toggle 'Teach' switch
     *  Click 'Teach TM5' or 'Teach TM12'
     *  Click 'Start' to launch the model of robot
     *  Use 'TCP Position' and 'TCP Orientation' to control the end of effector of robot
     *  Toggle 'JS' switch to control the robot by Joint Position
     *  Toggle 'TCP' switch to control the robot by input the position of end effector
    
* Note: for real demonstration, please refer to our omcron_package

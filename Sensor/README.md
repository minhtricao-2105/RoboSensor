# TurtleBot Following Each Other Project

## Description:
In this innovative project, we aim to create an autonomous system where a TurtleBot, termed as the "follower," uses vision-based techniques to tail another TurtleBot, known as the "guider." Harnessing the capabilities of the OpenCV library, the follower detects ArUco markers strategically placed on the guider. These visual cues allow the follower to determine the position and orientation of the guider robot in real-time.

Using advanced control algorithms, the follower bot not only keeps track of the guider's location but also adjusts its movement patterns to maintain a consistent distance and alignment with it. If the OpenCV's default capabilities fall short in any aspect, supplementary control techniques are integrated, optimizing the bot's tracking mechanism.

Furthermore, the project environment is simulated within Gazebo, resembling a room setting with furniture, to mirror real-world conditions. A key feature implemented is the ability to control the guider TurtleBot using both a keyboard and an Xbox joystick. This dual-control mechanism ensures flexibility in testing and fine-tuning the follower's responses.

Challenges faced, such as intermittent ArUco marker detection and potential collision scenarios, have been addressed by enhancing detection techniques and implementing obstacle avoidance measures. The project serves as a testament to the application of computer vision and robotics control in creating synchronized and autonomous multi-robot systems. As we continue refining our algorithms and strategies, we anticipate achieving seamless tandem movement between the guider and follower TurtleBots, both in simulation and real-world scenarios.

## Contributors:
The contributors in this project are:
  1.  Minh Tri Cao - Student ID Number: 14231702:
    *  Creating world file.
    *  Control the guider.
    *  Apply P Controller for the Follower.
    *  Testing algorithm in the acutal TurtleBot.
      
  2.  Anh Tung Nguyen - Student ID Number: 14072286:
    *  Creating launch file.
    *  Control the follower.
    *  Apply Line of best fit Approach for the Follower.
    *  Testing algorithm in the acutal TurtleBot.

## Installation and Setup Guide
### Simulation Environment
  1. Prerequisites:
  Before diving into the project setup, ensure you have the following prerequisites installed:****
     *  ROS (Robot Operating System): Our project heavily relies on ROS functionalities, we refer ROS Noetic with Ubuntu 20.04.
     *  Gazebo: The primary simulation tool we use.
     *  Python (preferably Python 3): Essential for running our scripts and interfacing with hardware components.
     *  OpenCV: For vision-based techniques and ArUco marker detection.
  Note: Detailed installation instructions for these tools can be found on their respective official websites.

  2. Control Setup
    * Keyboard Control:
       *  Using these keys: 'W', 'A', 'S', 'D', 'Space' to control the guider turtlebot, detailed can be found in the script of guider_keyboard.py
    *  Xbox Joystick Control:
      *  Ensure your Xbox joystick is correctly connected and detected by your machine.
      *  Use the joystick's analog sticks and buttons to guide the guider TurtleBot's movement. A detailed button map is available in the joystick control guide. (guider_controller.py)
    
  4. Running the Project:
  *  In the first terminal, Launching the Simulation:
  ```
  roslaunch double_turtlebot_launcher multi_turtlebot3.launch
  ```

  *  In another terminal, run the script to control the guider by keyboard:
  ```
  rosrun turtlebot3_double guider_keyboard.py
  ```

  *  In another terminal, run the script of the follower turtlebot:
  
  ```
  rosrun turtlebot3_double follower.py
  ```

### Setup in Real TurtleBot

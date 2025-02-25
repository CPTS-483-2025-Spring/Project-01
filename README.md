## Project 01

Complete the following items, commit & push all your code, and create a report for the following tasks.

### Task 1
Install Ubuntu 22.04, and ROS2 Humble. You may use a virtual machine, such as VirtualBox on Windows, and install Ubuntu 22.04. Please use the exact version of Ubuntu and ROS2. Follow [this link](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) to install ROS2. Then take a screenshot of the "talker-listener" example and include it in your report. 
Note: If you are using VirtualBox, follow [this link](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview), and make sure to install Guest Additions. 

### Task 2
Complete the first set of beginner tutorials [here](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html) to understand basic ROS concepts. Then complete the following tasks:

 - In turtlesim, control three turtles to draw three red letters "W" "S" "and U" respectively, on a white background. Then take a screenshot and include it in your report. 

### Task 3
Complete the second set of beginner tutorials [here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html) to understand ROS packages. Then complete the following tasks:

 - Create your own package for service and massage interfaces, call it "cpts483_interface"
 - Create a service interface where the request and the response are both strings. Take a screenshot of the service and client terminals together and include it in your report. 

### Task 4
Check out [Launch](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html), [tf2](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html), [URDF](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html), and [RViz](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-Main.html) tutorials. Clone the this repo into `<your-ros-ws>/src`, Launch the `launch_franka.py` in `/launch` folder, and then complete the following tasks:

 - Create a diagram of frames, include this diagram in your report.
 - Add TF display in RViz to show all the frames.
 - Use the Joint State Publisher GUI to change the arm joint angles, then use `tf2_echo` to report the transformation from the `panda_link0` to `panda_ee`. Take a screenshot of the RViz window with the transformation and include it in your report. 

Complete the todos in the starter code `franka_fk.py` to compute the forward kinematics of the Franka arm end-effector. You can use `tf2_echo` to compare results. 

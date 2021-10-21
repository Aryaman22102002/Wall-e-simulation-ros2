# Wall-e-simulation-ros
A simulation of a self-balancing and line following bot.

## Table of Contents
* [About the Project](#about-the-project)
  * [Tech Stack](#tech-stack)
  * [Tools](#tools)
  * [File Structure](#file-structure)
* [Getting Started](#getting-started)
  * [Prerequisites](#prerequisites)
  * [Installation](#installation)
  * [Configuration](#configuration)
* [Usage](#usage)
* [Results and Demo](#results-and-demo)
* [Future Work](#future-work)
* [Troubleshooting](#troubleshooting)
* [Contributors](#contributors)
* [Acknowledgements and Resources](#acknowledgements-and-resources)
* [License](#license)

## About the Project

![Screenshot from 2021-10-16 01-21-54](https://user-images.githubusercontent.com/82901720/137546447-a77caa4a-a804-49ad-93dc-8da561bad422.png)
![Screenshot from 2021-10-16 01-22-21](https://user-images.githubusercontent.com/82901720/137546503-a18a2a31-550b-4782-88b2-aed310f0bb4f.png)

* The aim of the project was to design a bot and implement self-balancing and line-following algorithms on it.
* The designing of the bot is done using solidworks and the self-balancing and line-following have been achieved using PID.

### Tech Stack
* ROS2 Foxy

### Tools
* Gazebo Version: 11.0
* RViz
* SolidWorks
* Autocad
* MS Paint

### File Structure
```
   ┣ 📂config
   ┃ ┗ 📜joint_names_Wall-E-urdf1.yaml           # Configuration file for joints of the bot
   ┣ 📂launch                                    # All launch files
   ┃ ┣ 📜emptyworld.launch.py
   ┃ ┣ 📜gazebo.launch.py
   ┃ ┣ 📜gzclient.launch.py
   ┃ ┣ 📜gzserver.launch.py
   ┃ ┣ 📜line_following.launch.py                # Launch file for bot for line-following algorithm
   ┃ ┣ 📜robot_state_publisher.launch.py         # Launch file for bot for self-balancing algorithm
   ┃ ┣ 📜self_balancing.launch.py
   ┣ 📂meshes                                    # Meshes for different parts of bot
   ┃ ┣ 📜base_link.STL
   ┃ ┣ 📜chassis.STL
   ┃ ┣ 📜leftwheel.STL
   ┃ ┗ 📜rightwheel.STL
   ┣ 📂models
   ┃ ┣ 📜course.material                         # The file which links the world file to the png of the world
   ┃ ┗ 📜sra.png                                 # The png which decides the design of the world in Gazebo
   ┣ 📂rviz
   ┃ ┗ 📜urdf_config.rviz                        # The file for rviz configuration
   ┣ 📂src                                       # All cpp codes are stored here
   ┃ ┣ 📜line_following.cpp                      # The line-following algorithm
   ┃ ┣ 📜self_balancing.cpp                      # The sdf files for bot are stored here 
   ┣ 📂urdf
   ┃ ┣ 📜walle.csv
   ┃ ┣ 📜walle.urdf   
   ┃ ┣ 📜walle.sdf                               # The sdf file for self-balancing bot
   ┃ ┣ 📜walle2.sdf                              # The sdf file for line-following bot
   ┣ 📂worlds
   ┃ ┗ 📜sra.world                               # The line-following path
   ┣ 📜.gitignore
   ┣ 📜CMakeLists.txt                            # Contains all the information regarding the packages to be imported
   ┣ 📜LICENSE
   ┣ 📜README.md
   ┣ 📜export.log
   ┣ 📜package.xml                               # Contains all the information regarding the dependencies to be imported
```  
    
## Getting Started

### Prerequisites

    1. ROS2 Foxy
    2. Gazebo Version: 11.0
    3. RViz 
    
* You can find the link for installtion of ROS2 Foxy in video format [here](https://youtu.be/fxRWY0j3p_U) and in text format [here](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html). Note that the video is for installing ROS2 Foxy on Ubuntu whereas you can find the the instructions according to your 
own operating system in the official documentation provided in the second link above.
* The link to install Gazebo for Ubuntu is provided [here](http://gazebosim.org/tutorials?tut=install_ubuntu). You can find the instructions to install different versions of Gazebo in the link provided above but using Gazebo 11.0 is recommed for ROS2 Foxy.
* The link to install RViz on Ubunntu is provided [here](https://zoomadmin.com/HowToInstall/UbuntuPackage/rviz).

### Installation
1. Clone the repo
<code>
    git clone https://github.com/Aryaman22102002/Wall-e-simulation-ros.git
</code>

### Configuration
The colour of the bot can be varied by the changing the colour in the material tag of the visual element in the bot's sdf. The list of all colours and materials available for Gazebo are avialable [here](http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials).

## Usage
1. After cloning the repo, go into your ROS2 workspace and run the following commands:<br/>
<code>
colcon build       

. install/setup.bash
</code>  

2. Now enter into the Wall-e-simulation-ros folder.

3. For launching the self-balancing bot, enter the following command:<code>  ros2 launch my_bot self_balancing.launch.py  </code>
 
4. For running the self-balancing code, enter the following command:<code>  ros2 run my_bot self_balancing </code>
 
5. For launching the line-following bot, enter the following command:<code>  ros2 launch my_bot line_following.launch.py </code>
  
6. For running the line-following code, enter the following command:<code> ros2 run my_bot line_following  </code>

## Results and Demo
The implementation of the self balancing and line following codes has been demonstarted in the following video.

https://user-images.githubusercontent.com/82901720/137575995-79693d64-294f-4f40-88f0-2290bfa87bfe.mp4

## Future Work
- [ ] Combine self-balancing and line-following 
- [ ] Implement maze solving algorithms 

## Troubleshooting 
* Sometimes the self-balancing code will behave differently than the previous run. Try to stop the execution of the code and launch the bot again and run the code.
* Always launch the bot and the codes from within the Wall-e-simulation-ros folder. Otherwise it may not the launch the world correctly.
* Also sometimes Gazebo might not launch inspite of running the launch commands. In such cases, try entering the command <code>killall gzserver</code> and then launching the bot again.

## Contributors
* [Aryaman Shardul](https://github.com/Aryaman22102002)<br/>
* [Marck Koothoor](https://github.com/marck3131)

## Acknowledgements and Resources
* [SRA Vjti](https://www.sravjti.in/) Eklavya 2021<br/>
* Special thanks to our mentors [Gautam Agrawal](https://github.com/gautam-dev-maker) and [Anushree Sabnis](https://github.com/MOLOCH-dev).<br/>
* https://github.com/SRA-VJTI/Wall-E_v2.2
* https://navigation.ros.org/setup_guides/urdf/setup_urdf.html

## License
The [license](https://github.com/Aryaman22102002/Wall-e-simulation-ros/blob/main/LICENSE) used for this project.




  
      
 





 




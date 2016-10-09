# ROS tutorials

## 1. Navigating the ROS Filesystem
### (1). rospack 
```rospack``` allows you to get information about packages. In this tutorial, we are only going to cover the ```find``` option, which returns the path to package. 

Usage: 
```
$ rospack find [package_name]
```
### (2). roscd 
```roscd``` is part of the rosbash suite. It allows you to change directory (cd) directly to a package or a stack. 

Usage: 
```
$ roscd [locationname[/subdir]]
```
```roscd```can also move to a subdirectory of a package or stack. 

Try: 
```
$ roscd roscpp/cmake
```

### (3).roscd log
```roscd log``` will take you to the folder where ROS stores log files. Note that if you have not run any ROS programs yet, this will yield an error saying that it does not yet exist. 
If you have run some ROS program before, try: 
```
$ roscd log
```

### (4).rosls is part of the rosbash suite. It allows you to ls directly in a package by name rather than by absolute path. 
Usage: 
$ rosls [locationname[/subdir]]
Example: 
$ rosls roscpp_tutorials

##2. Creating a ROS Package
### (1). A catkin Package
The simplest possible package might have a structure which looks like this: 
my_package/
  CMakeLists.txt
  package.xml
### (2).A catkin Workspace
A trivial workspace might look like this: 
```
workspace_folder/        -- WORKSPACE
  src/                   -- SOURCE SPACE
    CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
    package_1/
      CMakeLists.txt     -- CMakeLists.txt file for package_1
      package.xml        -- Package manifest for package_1
    ...
    package_n/
      CMakeLists.txt     -- CMakeLists.txt file for package_n
      package.xml        -- Package manifest for package_n
```
Let's create a catkin workspace: 
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
```
Even though the workspace is empty (there are no packages in the 'src' folder, just a single CMakeLists.txt link) you can still "build" the workspace: 
```
$ cd ~/catkin_ws/
$ catkin_make
```
The catkin_make command is a convenience tool for working with catkin workspaces. 
If you look in your current directory you should now have a ```build``` and ```devel``` folder. Inside the ```devel``` folder you can see that there are now several setup```.*sh``` files. 

Sourcing any of these files will overlay this workspace on top of your environment. To understand more about this see the general catkin documentation: catkin. Before continuing source your new setup```.*sh``` file: 
```
$ source devel/setup.bash
```
To make sure your workspace is properly overlayed by the setup script, make sure ```ROS_PACKAGE_PATH``` environment variable includes the directory you're in. 
```
$ echo $ROS_PACKAGE_PATH
/home/youruser/catkin_ws/src:/opt/ros/kinetic/share:/opt/ros/kinetic/stacks
```
Use the ```catkin_create_pkg``` script to create a new package called ```beginner_tutorials``` which depends on ```std_msgs```, ```roscpp```, and ```rospy```:
```
$ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
```
### (3). package dependencies
When using ```catkin_create_pkg``` earlier, a few package dependencies were provided. These first-order dependencies can now be reviewed with the rospack tool. 
```
$ rospack depends1 beginner_tutorials 
```
### (4). Customizing Your Package
Customizing the package.xml
When needed, use website: http://wiki.ros.org/ROS/Tutorials/CreatingPackage

## 3. Building a ROS Package
As long as all of the system dependencies of your package are installed, we can now build your new package. 
```
$ source /opt/ros/%YOUR_ROS_DISTRO%/setup.bash
```
### ```catkin_make```
```catkin_make``` is a command line tool which adds some convenience to the standard catkin workflow. You can imagine that ```catkin_make``` combines the calls to ```cmake``` and ```make``` in the standard CMake workflow. 

Usage:
```
# In a catkin workspace
$ catkin_make [make_targets] [-DCMAKE_VARIABLES=...]
$ catkin_make install  # (optionally)
```
The above commands will build any catkin projects found in the ```src``` folder.
## 4. Understanding ROS Nodes
### Quick Overview of Graph Concepts
- Nodes: A node is an executable that uses ROS to communicate with other nodes. 
- Messages: ROS data type used when subscribing or publishing to a topic. 
- Topics: Nodes can publish messages to a topic as well as subscribe to a topic to receive messages. 
- Master: Name service for ROS (i.e. helps nodes find each other) 
- rosout: ROS equivalent of stdout/stderr 
- roscore: Master + rosout + parameter server (parameter server will be introduced later)
### (1). Nodes
A node really isn't much more than an executable file within a ROS package. ROS nodes use a ROS client library to communicate with other nodes. Nodes can publish or subscribe to a Topic. Nodes can also provide or use a Service. 
### (2).Client Libraries
ROS client libraries allow nodes written in different programming languages to communicate: 
- ```rospy``` = python client library 
- ```roscpp``` = c++ client library 
### (3). ```roscore```
```roscore``` is the first thing you should run when using ROS. 

Please run: 
```
$ roscore
```
### (4). ```rosnode list```
The ```rosnode list``` command lists these active nodes: 
```
$ rosnode list
```
### (5). ```rosnode info```
The ```rosnode info``` command returns information about a specific node. 
```
$ rosnode info /rosout
```
### (6).  ```rosrun```
```rosrun``` allows you to use the package name to directly run a node within a package (without having to know the package path). 

Usage: 
```
$ rosrun [package_name] [node_name]
```
## 4. Understanding ROS Topics
### (1). turtlesim
For this tutorial we will also use turtlesim. Please run in a new terminal: 
```
$ rosrun turtlesim turtlesim_node
```
### (2). turtle keyboard teleoperation
We'll also need something to drive the turtle around with. Please run in a new terminal: 
```
$ rosrun turtlesim turtle_teleop_key
[ INFO] 1254264546.878445000: Started node [/teleop_turtle], pid [5528], bound on [aqy], xmlrpc port [43918], tcpros port [55936], logging to [~/ros/ros/log/teleop_turtle_5528.log], using [real] time
Reading from keyboard
---------------------------
Use arrow keys to move the turtle.
```
Now you can use the arrow keys of the keyboard to drive the turtle around. 

If you can not drive the turtle select the terminal window of the ```turtle_teleop_key``` to make sure that the keys that you type are recorded. 

### (3). ROS Topics
The ```turtlesim_node``` and the ```turtle_teleop_key``` node are communicating with each other over a ROS Topic.
```turtle_teleop_key``` is publishing the key strokes on a topic, while ```turtlesim``` subscribes to the same topic to receive the key strokes. Let's use ```rqt_graph``` which shows the nodes and topics currently running. 
### (4). ```rqt_graph``` 
```rqt_graph``` creates a dynamic graph of what's going on in the system. ```rqt_graph``` is part of the rqt package. Unless you already have it installed, run: 
```
$ sudo apt-get install ros-<distro>-rqt
$ sudo apt-get install ros-<distro>-rqt-common-plugins
```
### (5). rostopic tool
The rostopic tool allows you to get information about ROS topics. 
You can use the ```help``` option to get the available sub-commands for rostopic 
```
$ rostopic -h
```
### (6). ```rostopic echo``` 
```rostopic echo``` shows the data published on a topic. 

Usage: 
```
rostopic echo [topic]
```
Example:
```
$ rostopic echo /turtle1/command_velocity
```
### (6).Using ```rostopic list```
```rostopic list``` returns a list of all topics currently subscribed to and published. 
Let's figure out what argument the list sub-command needs. In a new terminal run: 
```
$ rostopic list -h
```
Usage: 
```
rostopic list [/topic]

Options:
  -h, --help            show this help message and exit
  -b BAGFILE, --bag=BAGFILE
                        list topics in .bag file
  -v, --verbose         list full details about each topic
  -p                    list only publishers
  -s                    list only subscribers
  ```
### ROS Messages
Communication on topics happens by sending ROS messages between nodes. For the publisher ```turtle_teleop_key``` and subscriber ```turtlesim_node``` to communicate, the publisher and subscriber must send and receive the same type of message. This means that a topic type is defined by the message type published on it. The type of the message sent on a topic can be determined using ```rostopic type```.
### (7). Using ```rostopic type```
``rostopic type``` returns the message type of any topic being published. 

Usage: 
```
rostopic type [topic]
```
Example:
```
$ rostopic type /turtle1/cmd_vel
```
We can look at the details of the message using ```rosmsg```: 
```
$ rosmsg show geometry_msgs/Twist #geometry_msgs/Twist is a rostopic type.
```
### (8). Using ```rostopic pub```
```rostopic pub``` publishes data on to a topic currently advertised. 

Usage: 
```
rostopic pub [topic] [msg_type] [args]
```
Example: 
```
$ rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
```
- ```rostopic pub``` This command will publish messages to a given topic.
- ```-1 ``` This option (dash-one) causes rostopic to only publish one message then exit.
- ```/turtle1/cmd_vel``` This is the name of the topic to publish to.
- ```geometry_msgs/Twist``` This is the message type to use when publishing to the topic.
- ```--``` This option (double-dash) tells the option parser that none of the following arguments is an option. This is required in cases where your arguments have a leading dash -, like negative numbers.
- ```'[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]' ``` As noted before, a ```geometry_msgs/Twist msg``` has two vectors of three floating point elements each: linear and angular. In this case, ```'[2.0, 0.0, 0.0]'``` becomes the linear value with **x=2.0, y=0.0, and z=0.0**, and ```'[0.0, 0.0, 1.8]'``` is the angular value with **x=0.0, y=0.0, and z=1.8**. These arguments are actually in YAML syntax, which is described more in the YAML command line documentation. 

You may have noticed that the turtle has stopped moving; this is because the turtle requires a steady stream of commands at 1 Hz to keep moving. We can publish a steady stream of commands using ```rostopic pub -r``` command: 
```
$ rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
```
### (9). Using ```rostopic hz```
```rostopic hz``` reports the rate at which data is published. 

Usage: 
```
rostopic hz [topic]
```
Let's see how fast the ```turtlesim_node``` is publishing ```/turtle1/pose```: 
```
$ rostopic hz /turtle1/pose
```
### (10). Using ```rqt_plot```
```rqt_plot``` displays a scrolling time plot of the data published on topics. Here we'll use ```rqt_plot to plot``` the data being published on the ```/turtle1/pose``` topic. First, start ```rqt_plot``` by typing 
```
$ rosrun rqt_plot rqt_plot
```
Example:

in a new terminal. In the new window that should pop up, a text box in the upper left corner gives you the ability to add any topic to the plot. Typing ```/turtle1/pose/x``` will highlight the plus button, previously disabled. Press it and repeat the same procedure with the topic ```/turtle1/pose/y```. You will now see the turtle's x-y location plotted in the graph. 



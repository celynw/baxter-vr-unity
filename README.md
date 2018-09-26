# VR for Baxter in Unity

A VR interface for visualising and controlling the Baxter Robot in real time.
This utilises SteamVR with Unity and ROS.

This project was created in 2016/2017 for my third-year project, for Electronic Engineering with Computer Systems at the University of Surrey.

As a result, lots of this was created naïvely with very limited experience.
However, it still works and is used for live demos.
The code may be very loosely maintained but no further development is planned, although it could be fun I don't have time.

Please feel free to make issues, ask questions, fork, etc.!

## Demo

[![VR control for Baxter Robot with ROS in Unity](https://img.youtube.com/vi/jMmBaltZ3LY/0.jpg)](https://www.youtube.com/watch?v=jMmBaltZ3LY "VR control for Baxter Robot with ROS in Unity")

## Installation

This project consists of three parts:

1. ROS workspace for running on a linux computer connected to the Baxter robot
1. Unity project for interfacing in VR
1. Solution for a Windows library callable from Unity, which connects to the linux computer.

### ROS Workspace

An example workspace can be found in the `ros_ws` directory. The package is under `ros_ws/src/robotic_telepresence`

#### Prerequisites

Minimum:

- rosserial-server
- baxter stuff (e.g. baxter-interface)

Others:

- moveit
- pcl-ros
- openni-launch

### Unity Project and Library

The rosnode to communicate with the baxter computer must be compiled.

NOTE: Set your IP address of the baxter computer in the [main file](../master/rosnode_windows/ROSnode.cpp).

A command-line application is then called from the Unity game, with shared memory to transfer data between them.
The solution can be found in `rosnode_windows`, and makes use of [`rosserial_windows`](http://wiki.ros.org/rosserial_windows "ROS.org"). Good luck!

Open the Unity project, and open the `Main` scene.
On pressing play, the command-line application should open in the foreground.
It closes when the game closes.

## Setup

### Linux Computer

- Turn Baxter on
- Run roscore and set up baxter for running (i.e. untuck arms)

- Init:

```bash
cd /vol/vssp/baxter/ros/baxter_ws
. baxter.sh
cd ~/Documents/Baxter
. ./devel/setup.bash
rostopic pub /robot/sonar/head_sonar/set_sonars_enabled std_msgs/UInt16 0
rosrun baxter_tools enable_robot.py -e
rosrun baxter_tools tuck_arms.py -u
```

- Everything:

```bash
roslaunch robotic_telepresence server.launch
```

- OR Visualisation only:

```bash
rosrun rosserial_server socket_node
```

- Finish:

```bash
rosrun baxter_tools tuck_arms.py -t
rosrun baxter_tools enable_robot.py -d
```

- Turn off Baxter

### Windows Computer

Seems to be more reliable if followed:

1. Make sure Oculus and camera(s) are plugged in. One should work, two is better
1. Start Oculus program
1. Open SteamVR
1. Make sure headset, cameras, controllers are solidly highlighted in green
1. Open Unity and open my project "Robotic Telepresence"
1. Press the play button at the top
1. Black cmd should open. Don't close it, but you can hide it
    - May display error messages here
    - A common one is "Could not open non-blocking". Refer to "if something not working"
1. If the arms pop into their correct locations, it's working!

- If SteamVR won't open because you need to log in, then I think you can just play the Unity game, and a copy will automatically open. Might need a few tries this way though

## Controls

- Middle finger to grip. Has live feedback
- Index finger: Click and drag. On release, requests a move to this location. Careful of gripper orientation

## Important Notes (probably worth reading)

- If something isn't working:
  - Restart roslaunch on baxterpc
- Still not working?
  - Stop Unity game (cmd will close as well)
  - Restart roslaunch on baxterpc
  - Then re-play Unity

- Collisions for the table are not set!
    I used to do this by using "baxter_face.scene" and "baxter_table.scene".
    You can do this by going into RViz which opens.
    Add "Motion Planning" panel -> "Scene objects" -> "Import File".
    These are now probably out of date! But may be useful. Look at the visualisation

- roslaunch takes a long time to close, but it should close on error ("required" is set)

- Yes, you can't see the grippers open or close in VR. You can hear them though...

- Maximising RViz seems to crash it. Why is it so unreliable on this computer?!

- If VR seems to be paused or black screen, you might need the "Game" tab open and focused in Unity

- If you need to move virtual Baxter, I have a Unity GameObject called TRANSFORM on the top level.

- The headset needs to be tracking (not flashing green) in order to enable other things...

- Making a change to Unity when in game mode is TEMPORARY and will be lost when stopped. Pain in the ass for getting the point cloud transform

- Kinect cable won't reach PC! So I haven't tested it. But it's possible it will just work!

## Contact

Celyn Walters `c.walters@surrey.ac.uk`

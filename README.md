# CDT Husky Challenge

## Introduction

This repo hosts all of the code for the CDT husky challenge. It is composed of the following repositories:

* `SLAM`: This repository contains a basic implementation of SLAM
* `sensors`: Provides various helper functions for reading in and visualising data
received from the huskies into MATLAB. For more information see `examples/sensor_example.m`
* `control`: Provides functions for controlling the husky using MATLAB. For more information see
`example/move_robot_example.m`.

And that is pretty much it! Before using this repository make sure you read the rest of this README...

## Setup
### Assumptions
From this point on it is assumed that you have the following
dependencies installed:

* A C++ compiler (eg XCode)
* Git
* A departmental install of MATLAB (> R2016b)

### MOOS and MEXMOOS
Moos acts as a message passing interface between the huskies
and your local machines and exists as an open source project
hosted on `GitHub`. Mexmoos allows messages to be received
and sent over MOOS from MATLAB.

In order to install these packages we we will use `cmake` which
can be installed using homebrew. If you do **not** have
homebrew installed on your local machine it can be installed
using:

```buildoutcfg
ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```

Now install cmake as:
```buildoutcfg
brew install cmake
```

Moos can now be installed as follows
```buildoutcfg
mkdir -p ~/code/core-moos-v10/build
cd ~/code/core-moos-v10
git clone https://github.com/themoos/core-moos.git src
cd src
git checkout 10.0.2-release
cd ../build
cmake ../src
make -j
```

**Note, it is essential that you checkout version `10.0.2` as this is the only version
currently supported by Mexmoos. You may get a warning that you are in a detached HEAD state; you can ignore this.**

Next install mexmoos...
```buildoutcfg
mkdir -p ~/code/mex-moos/build
cd ~/code/mex-moos
git clone https://github.com/themoos/mex-moos.git src
cd ../build
cmake ../src
```

Now check that the correct paths have been found. Type in

```buildoutcfg
ccmake .
```
and ensure that the following paths are set to something like:
```buildoutcfg
 MATLAB_ROOT  /Applications/MATLAB_R2016b.app       # Your Matlab application path
 MOOS_DIR  /Users/robweston/code/core-moos-v10/build    # Your core-moos build folder
```
At this point press c to configure and g to generate before building as

```buildoutcfg
make -j
```

You should now have Mexmoos installed. The final step is to add the following paths
to the `startup-cdt.m` script found in this folder.

```buildoutcfg
% --> CHANGE THESE PATHS TO YOUR USER
addpath('/Users/robweston/code/mexmoos/src')
addpath('/Users/robweston/code/mexmoos/build')
```
To check that everything has installed correctly, either (1) run your `startup-cdt.m` manually followed by `examples/simple_moos_example.m`. This script should display the following:

```
>> simple_moos_example
closing MOOS Comms... done
* mexmoos initialising *
Property MOOSNAME                   Str: ExampleCdtClient9057919
Property SERVERHOST                 Str: 192.168.0.14
Property SERVERPORT                 Str: 9000
DB connection established
Sending message over MOOS with unique number: 9057919
Reading message from MOOS
Received Message:  "Hello, MOOS world (9057919)"
closing MOOS Comms... done
```

Or (2) close and re-open MATLAB and type into the prompt: `mexmoos`. If everything is correctly installed, this should print out some mexmoos usage instructions.

## Using the Huskies
All of your interactions with the huskies happen through either `MissionControl` or through `mexmoos` and a set of provided
`MATLAB` functions. `MissionControl` is a tool developed by ORI which provides an easy way to launch and
manage multiple tasks on mobile platforms.

### Initial Set-Up
In order to use the huskies:

1) Get a charged husky battery
2) Install the battery in the back of the husky, using a screwdriver to open the panel if required
3) Turn on the husky and and take note of the battery meter - there should be red lights corresponding to
the e-stop and the lockout key. Wait a few seconds for the comms light to go from red to yellow
4) Connect to HuskyX's WiFi network (password: CDTrobots)
5) Open a browser and enter "192.168.0.14:8080" to open `MissionControl` where you can start to launch tasks

### Receiving data from the Huskies
Now click on the control icon (gear) and you should see various different task groups. The `Moos` and `StartUpTasks`
should be highlighted green. To launch the camera on the huskie go to `sensors -> Bumblebee2_NoLogs` and click run.
Now do the same for the laser. Both `Bumblebee2_NoLogs` and `LMS1xxPublishOnly`
should now be green on the `sensors` task page.
If this is not the case you may need to reboot your system. Go to `general -> Reboot_Machine` and
click run before re-launching the camera and laser drivers. It is often helpful to check whether the drivers
are publishing over MOOS. To check this go to `Moos -> mtm` and click run. `mtm` (Monitor The Moos) is an application
which outputs various information about the channels currently registered with `Moos`. Your output should look
something like:

```buildoutcfg
                   BUMBLEBEE2_IMAGES    17:48:53.586         /home/mrg/code/mrgcorelibs/build/base-cpp/bin/pgr_
DB_CLIENTS           17:48:53.768         MOOSDB_#1            0.5 Hz S  iLMS1xx-14320092,mtm-1...
DB_QOS               17:48:51.767         MOOSDB_#1            0.5 Hz S  /home/mrg/code/mrgcore...
DB_RWSUMMARY         17:48:51.767         MOOSDB_#1            0.5 Hz S  iLMS1xx-14320092=&LMS1...
DB_TIME              17:48:53.768         MOOSDB_#1            0.5 Hz D  1519753733.8
DB_UPTIME            17:48:53.768         MOOSDB_#1            0.5 Hz D  12050.8
DB_VARSUMMARY        17:48:51.767         MOOSDB_#1            0.5 Hz S  BB2-14200208_pose_pb 0...
                   LMS1xx_14320092_laser2d 17:48:53.749         iLMS1xx-14320092     49.9Hz B  *binary* 9.573 KB
MTM-158_STATUS       17:48:53.767         mtm-158              0.5 Hz S  AppErrorFlag=false,Upt...
                   husky_odometry_pose  16:47:26.760         huskysrs             33210.2Hz B  *binary* 202  B
                   husky_state          16:47:26.760         huskysrs             33186.5Hz B  *binary* 111  B
```

The two channels `BUMBLEBEE2_IMAGES` and `LMS1xx_14320092_laser2d` correspond to the stereo camera and laser
respectively.

Data can be read into matlab using mexmoos. See the file `examples/sensors_example.m` for more information.

### Making the huskies move
#### Manually with a gamespad
In order to make the huskies move you first need to connect a gamespad to the huskie. To do this press
check that gamepad colour pattern matches the colour pattern on the USB dongle attached to the husky and
press the logitech button on the front of the gamepad until the gamepad lights up green (not flashing). This means that the gamepad is connected.

To drive the Husky manually, press the "Mode" button, and the green light should turn off (with the green light on you have to use the D-Pad to control the robot). Once the mode light is off (not flashing), press and hold down the deadman (RB top-right trigger). Now, you can move the Husky. The left toggle will now drive the robot forwards and backwards. The right toggle will turn it.

Note, the huskies will only move when they are connected to battery power and not to the mains.

#### With Matlab
The function `SendSpeedCommand` allows you to control the husky via MATLAB. In order
to use this function you will first need to launch autonomy mode on the huskies. To do this press 'Y' and then 'A'.
See the script `move_robot_example.m`

### Turning the Huskies Off
To shutdown the huskies click stop all taks and then run the task `general -> shutdown`. Now press the power button on the huskie!



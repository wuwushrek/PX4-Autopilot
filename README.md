# PX4 Drone Autopilot

[![Releases](https://img.shields.io/github/release/PX4/PX4-Autopilot.svg)](https://github.com/PX4/PX4-Autopilot/releases) [![DOI](https://zenodo.org/badge/22634/PX4/PX4-Autopilot.svg)](https://zenodo.org/badge/latestdoi/22634/PX4/PX4-Autopilot)

[![Nuttx Targets](https://github.com/PX4/PX4-Autopilot/workflows/Nuttx%20Targets/badge.svg)](https://github.com/PX4/PX4-Autopilot/actions?query=workflow%3A%22Nuttx+Targets%22?branch=master) [![SITL Tests](https://github.com/PX4/PX4-Autopilot/workflows/SITL%20Tests/badge.svg?branch=master)](https://github.com/PX4/PX4-Autopilot/actions?query=workflow%3A%22SITL+Tests%22)

[![Slack](/.github/slack.svg)](https://join.slack.com/t/px4/shared_invite/zt-si4xo5qs-R4baYFmMjlrT4rQK5yUnaA)

This repository holds the [PX4](http://px4.io) flight control solution for drones, with the main applications located in the [src/modules](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules) directory. It also contains the PX4 Drone Middleware Platform, which provides drivers and middleware to run drones.

PX4 is highly portable, OS-independent and supports Linux, NuttX and MacOS out of the box.

* Official Website: http://px4.io (License: BSD 3-clause, [LICENSE](https://github.com/PX4/PX4-Autopilot/blob/main/LICENSE))
* [Supported airframes](https://docs.px4.io/main/en/airframes/airframe_reference.html) ([portfolio](https://px4.io/ecosystem/commercial-systems/)):
  * [Multicopters](https://docs.px4.io/main/en/frames_multicopter/)
  * [Fixed wing](https://docs.px4.io/main/en/frames_plane/)
  * [VTOL](https://docs.px4.io/main/en/frames_vtol/)
  * [Autogyro](https://docs.px4.io/main/en/frames_autogyro/)
  * [Rover](https://docs.px4.io/main/en/frames_rover/)
  * many more experimental types (Blimps, Boats, Submarines, High altitude balloons, etc)
* Releases: [Downloads](https://github.com/PX4/PX4-Autopilot/releases)


## Building a PX4 based drone, rover, boat or robot

The [PX4 User Guide](https://docs.px4.io/main/en/) explains how to assemble [supported vehicles](https://docs.px4.io/main/en/airframes/airframe_reference.html) and fly drones with PX4.
See the [forum and chat](https://docs.px4.io/main/en/#getting-help) if you need help!


## MPC Modifications in mpc_franck branch

Now, we are going to explain the modifications we have made in the mpc_franck branch.

1. Checkout the mpc_franck branch, assuming you are in the PX4-Autopilot directory:
```bash
cd ~/Documents/
git clone https://github.com/wuwushrek/PX4-Autopilot.git
cd PX4-Autopilot
git checkout mpc_franck
git submodule update --init --recursive
# bash ./Tools/setup/ubuntu.sh # If you have not installed the dependencies yet, https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html
```

2. We add the custom MPC messages in mavlink
```bash
code src/modules/mavlink/mavlink/message_definitions/v1.0/common.xml
```

Now, we will modify the .xml file by inserting the snipplet below
Search for SMART_BATTERY_INFO and copy and paste the tag below before the SMART_BATTERY_INFO message and save the file.
```xml
    <message id="367" name="MPC_FULL_STATE">
      <description>Full MPC State used for offline mpc-based control</description>
      <field type="uint64_t" name="time_usec" units="us">Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.</field>
      <field type="float" name="x" units="m">X Position in POSE_FRAME_NED</field>
      <field type="float" name="y" units="m">Y Position in POSE_FRAME_NED</field>
      <field type="float" name="z" units="m">Z Position in POSE_FRAME_NED</field>
      <field type="float" name="vx" units="m/s">X Speed in POSE_FRAME_NED</field>
      <field type="float" name="vy" units="m/s">Y Speed in POSE_FRAME_NED</field>
      <field type="float" name="vz" units="m/s">Z Speed in POSE_FRAME_NED</field>
      <field type="float" name="qw">Qw quaternion : Body frame NED to pose_frame_ned</field>
      <field type="float" name="qx">Qx quaternion : Body frame NED to pose_frame_ned</field>
      <field type="float" name="qy">Qy quaternion : Body frame NED to pose_frame_ned</field>
      <field type="float" name="qz">Qz quaternion : Body frame NED to pose_frame_ned</field>
      <field type="float" name="wx" units="rad/s/s">Rollspeed : Body frame NED</field>
      <field type="float" name="wy" units="rad/s/s">Pitchspeed : Body frame NED</field>
      <field type="float" name="wz" units="rad/s/s">Yawspeed : Body frame NED</field>
      <field type="float" name="m1">Motor 1 input</field>
      <field type="float" name="m2">Motor 2 input</field>
      <field type="float" name="m3">Motor 3 input</field>
      <field type="float" name="m4">Motor 4 input</field>
      <field type="float" name="m5">Motor 5 input</field>
      <field type="float" name="m6">Motor 6 input</field>
    </message>
    <message id="368" name="MPC_MOTORS_CMD">
      <description>Full MPC Normalized motors commands</description>
      <field type="uint64_t" name="time_usec" units="us">Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.</field>
      <field type="float[6]" name="motor_val_des">Motor imputs between 0 and 1</field>
      <field type="float[4]" name="thrust_and_angrate_des">Thrust and angular rate desired command. T, wx, wy,wz. T is between 0 and 1 while wx, wy, wz have standard units</field>
      <field type="uint8_t" name="mpc_on">Specify if the state of the mpc</field>
      <field type="uint8_t" name="weight_motors">Constant parameter weighting if we track the angrate_des or motor output directly.Value between 0 and 100 </field>
    </message>
```

3. We add our light hexacopter model that doesn't have a gimbal and camera in the airframe file
```bash
cd ~/Documents/PX4-Autopilot
cp -r my_custom_models/* Tools/simulation/gazebo/sitl_gazebo/models/
```

4. Build the firmware for sitl for testing
```bash
make px4_sitl gazebo # Or, make px4_sitl gazebo_myhexa
```


========================================================================================================================
## Changing code and contributing

This [Developer Guide](https://docs.px4.io/main/en/development/development.html) is for software developers who want to modify the flight stack and middleware (e.g. to add new flight modes), hardware integrators who want to support new flight controller boards and peripherals, and anyone who wants to get PX4 working on a new (unsupported) airframe/vehicle.

Developers should read the [Guide for Contributions](https://docs.px4.io/main/en/contribute/).
See the [forum and chat](https://docs.px4.io/main/en/#getting-help) if you need help!


### Weekly Dev Call

The PX4 Dev Team syncs up on a [weekly dev call](https://docs.px4.io/main/en/contribute/).

> **Note** The dev call is open to all interested developers (not just the core dev team). This is a great opportunity to meet the team and contribute to the ongoing development of the platform. It includes a QA session for newcomers. All regular calls are listed in the [Dronecode calendar](https://www.dronecode.org/calendar/).


## Maintenance Team

  * Project: Founder
    * [Lorenz Meier](https://github.com/LorenzMeier)
  * Architecture
    * [Daniel Agar](https://github.com/dagar)
  * [Dev Call](https://github.com/PX4/PX4-Autopilot/labels/devcall)
    * [Ramon Roche](https://github.com/mrpollo)
  * Communication Architecture
    * [Beat Kueng](https://github.com/bkueng)
    * [Julian Oes](https://github.com/JulianOes)
  * UI in QGroundControl
    * [Gus Grubba](https://github.com/dogmaphobic)
  * [Multicopter Flight Control](https://github.com/PX4/PX4-Autopilot/labels/multicopter)
    * [Mathieu Bresciani](https://github.com/bresch)
  * [Multicopter Software Architecture](https://github.com/PX4/PX4-Autopilot/labels/multicopter)
    * [Matthias Grob](https://github.com/MaEtUgR)
  * [VTOL Flight Control](https://github.com/PX4/PX4-Autopilot/labels/vtol)
    * [Roman Bapst](https://github.com/RomanBapst)
  * [Fixed Wing Flight Control](https://github.com/PX4/PX4-Autopilot/labels/fixedwing)
    * [Roman Bapst](https://github.com/RomanBapst)
  * OS / NuttX
    * [David Sidrane](https://github.com/davids5)
  * Driver Architecture
    * [Daniel Agar](https://github.com/dagar)
  * Commander Architecture
    * [Julian Oes](https://github.com/julianoes)
  * [UAVCAN](https://github.com/PX4/PX4-Autopilot/labels/uavcan)
    * [Daniel Agar](https://github.com/dagar)
  * [State Estimation](https://github.com/PX4/PX4-Autopilot/issues?q=is%3Aopen+is%3Aissue+label%3A%22state+estimation%22)
    * [Paul Riseborough](https://github.com/priseborough)
  * Vision based navigation and Obstacle Avoidance
    * [Markus Achtelik](https://github.com/markusachtelik)
  * RTPS/ROS2 Interface
    * [Nuno Marques](https://github.com/TSC21)

See also [maintainers list](https://px4.io/community/maintainers/) (px4.io) and the [contributors list](https://github.com/PX4/PX4-Autopilot/graphs/contributors) (Github).

## Supported Hardware

This repository contains code supporting Pixhawk standard boards (best supported, best tested, recommended choice) and proprietary boards.

### Pixhawk Standard Boards
  * FMUv6X and FMUv6U (STM32H7, 2021)
    * Various vendors will provide FMUv6X and FMUv6U based designs Q3/2021
  * FMUv5 and FMUv5X (STM32F7, 2019/20)
    * [Pixhawk 4 (FMUv5)](https://docs.px4.io/main/en/flight_controller/pixhawk4.html)
    * [Pixhawk 4 mini (FMUv5)](https://docs.px4.io/main/en/flight_controller/pixhawk4_mini.html)
    * [CUAV V5+ (FMUv5)](https://docs.px4.io/main/en/flight_controller/cuav_v5_plus.html)
    * [CUAV V5 nano (FMUv5)](https://docs.px4.io/main/en/flight_controller/cuav_v5_nano.html)
    * [Auterion Skynode (FMUv5X)](https://docs.auterion.com/skynode)
  * FMUv4 (STM32F4, 2015)
    * [Pixracer](https://docs.px4.io/main/en/flight_controller/pixracer.html)
    * [Pixhawk 3 Pro](https://docs.px4.io/main/en/flight_controller/pixhawk3_pro.html)
  * FMUv3 (STM32F4, 2014)
    * [Pixhawk 2](https://docs.px4.io/main/en/flight_controller/pixhawk-2.html)
    * [Pixhawk Mini](https://docs.px4.io/main/en/flight_controller/pixhawk_mini.html)
    * [CUAV Pixhack v3](https://docs.px4.io/main/en/flight_controller/pixhack_v3.html)
  * FMUv2 (STM32F4, 2013)
    * [Pixhawk](https://docs.px4.io/main/en/flight_controller/pixhawk.html)
    * [Pixfalcon](https://docs.px4.io/main/en/flight_controller/pixfalcon.html)

### Manufacturer and Community supported
  * [Holybro Durandal](https://docs.px4.io/main/en/flight_controller/durandal.html)
  * [Hex Cube Orange](https://docs.px4.io/main/en/flight_controller/cubepilot_cube_orange.html)
  * [Hex Cube Yellow](https://docs.px4.io/main/en/flight_controller/cubepilot_cube_yellow.html)
  * [Airmind MindPX V2.8](http://www.mindpx.net/assets/accessories/UserGuide_MindPX.pdf)
  * [Airmind MindRacer V1.2](http://mindpx.net/assets/accessories/mindracer_user_guide_v1.2.pdf)
  * [Bitcraze Crazyflie 2.0](https://docs.px4.io/main/en/complete_vehicles/crazyflie2.html)
  * [Omnibus F4 SD](https://docs.px4.io/main/en/flight_controller/omnibus_f4_sd.html)
  * [Holybro Kakute F7](https://docs.px4.io/main/en/flight_controller/kakutef7.html)
  * [Raspberry PI with Navio 2](https://docs.px4.io/main/en/flight_controller/raspberry_pi_navio2.html)

Additional information about supported hardware can be found in [PX4 user Guide > Autopilot Hardware](https://docs.px4.io/main/en/flight_controller/).

## Project Roadmap

A high level project roadmap is available [here](https://github.com/orgs/PX4/projects/25).

## Project Governance

The PX4 Autopilot project including all of its trademarks is hosted under [Dronecode](https://www.dronecode.org/), part of the Linux Foundation.

<a href="https://www.dronecode.org/" style="padding:20px" ><img src="https://mavlink.io/assets/site/logo_dronecode.png" alt="Dronecode Logo" width="110px"/></a>
<a href="https://www.linuxfoundation.org/projects" style="padding:20px;"><img src="https://mavlink.io/assets/site/logo_linux_foundation.png" alt="Linux Foundation Logo" width="80px" /></a>
<div style="padding:10px">&nbsp;</div>

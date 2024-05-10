# mavlink_natnet_bridge

Bridge between MAVLink (PX4 autopilot) and NatNet (OptiTrack mocap) for position control of drones flying indoor. 

This program is typically executed from within [Windows Subsystem for Linux (WSL)](https://learn.microsoft.com/en-us/windows/wsl/install) of a Windows machine running [Motive 3.1.0](https://optitrack.com/support/downloads/motive.html) and [QGroundControl](https://github.com/mavlink/qgroundcontrol/releases/tag/v4.3.0).

## Configure

Program configuration settings can be found as C++ defines in [mavlink_natnet_bridge.cpp](mavlink_natnet_bridge.cpp)

</br>
Motive Settings:

```
Edit -> Settings -> Streaming -> NatNet:
  Enable: True
  Local Interface: loopback
  Transmission Type: Unicast
  Up Axis: Z-Axis
  Disable all sliders except "Rigid Bodies"
```

</br>
QGroundControl Settings:

```
Application Settings -> General -> AutoConnect:
  UDP: DISABLE
Application Settings -> Comm Links -> Add:
  Name: mavlink_natnet_bridge
  Type: UDP
  Port: 14560
  Server: 127.0.0.1:14560
```

## Install

```
git clone git@github.com:mruggia/mavlink_natnet_bridge.git
cd mavlink_natnet_bridge
./install_dependencies.sh
cmake -Bbuild -H.
cmake --build build
```

## Run

```
cd mavlink_natnet_bridge/build
./mavlink_natnet_bridge
```
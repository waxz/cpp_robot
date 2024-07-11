# cpp_robot

There are libraries for ros1 and fastdds communication, and pallet detection.
- [unified message layer for ros1&&fastdds library](message_center)
- [pallet detection library](perception)
- [dds ros bridge demo](demo/dds_ros_bridge.cpp)
- [pallet detection demo](demo/pallet_detector.cpp)

# rust

The c binding of [unified message layer for ros1&&fastdds library](message_center) and [pallet detection library](perception) are used in [rs_robot](https://github.com/waxz/rs_robot).

# dependencies
### for fastdds
```shell
sudo apt install libasio-dev libssl-dev
```

### for ros1 built from source
```shell
sudo apt install libboost-thread-dev libboost-filesystem-dev
```
### fot optim
```shell
sudo apt-get install libblas-dev liblapack-dev
```
# usage
### compile
```shell
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=./install
```

### dds_ros_bridge demo
this demo is used to bridge ros1 layer and fastdds layer
```shell
cd build/bin
dds_ros_bridge  -r bridge_ros_config.toml -d bridge_dds_config.toml -b bridge_channel_config.toml
```

### pallet detection demo
The depth camera ros driver publishes pointcloud data.
But this demo uses fastdds layer, so dds_ros_bridge is needed.
```shell
cd build/bin
pallet_detector -d gui_config.toml
```



# terrainLoaderPlugin

### Commands in hand
```
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/uav/Documents/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/uav/Documents/terrainLoaderPlugin/build/lib

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/uav/Documents/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/uav/Desktop/sdf_maker

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/uav/Documents/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/uav/Documents/terrainLoaderPlugin/build/lib

gzserver /home/uav/Documents/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world --verbose

gz model --verbose --spawn-file="/home/uav/Documents/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/typhoon_h480/typhoon_h480.sdf" --model-name=typhoon_h480 -x 1.01 -y 0.98 -z 0.83

```
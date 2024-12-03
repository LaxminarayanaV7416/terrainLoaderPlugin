export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/uav/Documents/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/uav/Documents/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/uav/Documents/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic

export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/uav/Documents/terrainLoaderPlugin/build/lib
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/uav/Documents/terrainLoaderPlugin/models
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/uav/Documents/terrainLoaderPlugin/build/lib


# gzserver /home/uav/Documents/terrainLoaderPlugin/worlds/terrainLoader.world --verbose

# gzserver /home/uav/Documents/terrainLoaderPlugin/worlds/allPluginsWorld.world --verbose

gzserver /home/uav/Documents/terrainLoaderPlugin/worlds/windy.world --verbose

# gzserver /home/uav/Documents/terrainLoaderPlugin/worlds/empty.world --verbose

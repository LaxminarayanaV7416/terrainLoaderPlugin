export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/uav/Documents/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/uav/Documents/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/uav/Documents/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic

export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/uav/Documents/terrainLoaderPlugin/build/lib
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/uav/Documents/terrainLoaderPlugin/models
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/uav/Documents/terrainLoaderPlugin/build/lib


gz model --verbose --spawn-file="/home/uav/Documents/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/typhoon_h480/typhoon_h480.sdf" --model-name=typhoon -x 0 -y 0 -z 1.2
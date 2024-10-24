#ifndef GAZEBO_TERRAIN_LOADER_H
#define GAZEBO_TERRAIN_LOADER_H

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <time.h>
#include <map>
#include <list>
#include <iostream>

namespace gazebo {

class GazeboTerrainLoaderPlugin : public WorldPlugin {
public:
    GazeboTerrainLoaderPlugin();
    void Load(physics::WorldPtr _model, sdf::ElementPtr _sdf) override;
    
private:
    void On_msg(ConstPosesStampedPtr& _msg);
    void bringUpStaticBlockof1Meter(int& x_position, int& y_position, int& z_position);

private:
    transport::SubscriberPtr sub;
    msgs::Factory msg;
    transport::PublisherPtr publisher;
    time_t seconds = time (NULL);
};

} // namespace gazebo

#endif // GAZEBO_TERRAIN_LOADER_H

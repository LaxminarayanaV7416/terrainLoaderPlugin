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

namespace gazebo
{

    

    class GazeboTerrainLoaderPlugin : public WorldPlugin
    {
    public:
        GazeboTerrainLoaderPlugin()
        : WorldPlugin()
        {}        
        
        virtual ~GazeboTerrainLoaderPlugin();
        void Load(physics::WorldPtr _model, sdf::ElementPtr _sdf) override;

    private:
        void On_msg(ConstPosesStampedPtr &_msg);
        void bringUpStaticBlockof1Meter(int &x_position, int &y_position, double &z_position);

    private:
        transport::SubscriberPtr sub;
        msgs::Factory msg;
        transport::PublisherPtr publisher;
        time_t seconds = time(NULL);
        struct PairHash
        {
            template <class T1, class T2>
            std::size_t operator()(const std::pair<T1, T2> &pair) const
            {
                auto hash1 = std::hash<T1>{}(pair.first);
                auto hash2 = std::hash<T2>{}(pair.second);
                return hash1 ^ (hash2 << 1); // Combine the two hash values
            }
        };
        // defining the unordered_map for the reference to keep already spawned blocks
        std::unordered_map<std::pair<int, int>, double, PairHash> already_spawned_blocks_map;
    };

} // namespace gazebo

#endif // GAZEBO_TERRAIN_LOADER_H

#include "gazebo_terrain_loader.h"

namespace gazebo
{

  GazeboTerrainLoaderPlugin::GazeboTerrainLoaderPlugin() : WorldPlugin()
  {
    printf("Subscriber Plugin Created!\n");
    // model to use
    this->msg.set_sdf_filename("model://terrainLoaderBlock");
  }

  void GazeboTerrainLoaderPlugin::Load(physics::WorldPtr _model, sdf::ElementPtr _sdf)
  {

    // using _model pointer & GetName()to get model name
    std::cout << "World Name = " << _model->Name() << std::endl;

    // set a node to subscribe
    transport::NodePtr node(new transport::Node());
    node->Init();

    std::map<std::string, std::list<std::string>> topicsList = transport::getAdvertisedTopics();
    std::cout << "topic number" << topicsList.size() << std::endl;

    // subscribe to topic
    // transport::SubscriberPtr subscribe
    this->sub = node->Subscribe("~/pose/local/info", &GazeboTerrainLoaderPlugin::On_msg, this);

    // set publisher
    this->publisher = node->Advertise<msgs::Factory>("~/factory");
  }

  void GazeboTerrainLoaderPlugin::On_msg(ConstPosesStampedPtr &_msg)
  {

    // Dump the message contents to stdout.
    // std::cout << _msg->DebugString();

    // execute for every second
    time_t tempSeconds = time(NULL);

    if (tempSeconds > this->seconds)
    {
      this->seconds = tempSeconds;

      for (int i = 0; i < _msg->pose_size(); i++)
      {
        std::string msg_name = _msg->pose(i).name();

        // this is the drone pose
        if (msg_name.find("::") == std::string::npos)
        {
          double x = _msg->pose(i).position().x();
          double y = _msg->pose(i).position().y();
          double z = _msg->pose(i).position().z();

          int int_x = (int)x;
          int int_y = (int)y;
          int int_z = (int)z;

          // before spawning check whether the spawned block is already part of the
          this->bringUpStaticBlockof1Meter(int_x, int_y, int_z);
        }
      }
    }
  }

  void GazeboTerrainLoaderPlugin::bringUpStaticBlockof1Meter(int &x_position, int &y_position, int &z_position)
  {
    // set model pose
    msgs::Set(this->msg.mutable_pose(), ignition::math::Pose3d(x_position, y_position, 0, 0, 0, 0));

    // Send the message
    this->publisher->Publish(this->msg);

    std::cout << "spawning the plane at position " << x_position << " - Xposition and " << y_position << " - Yposition!" << std::endl;
  }

  // Register plugin
  GZ_REGISTER_WORLD_PLUGIN(GazeboTerrainLoaderPlugin)
};
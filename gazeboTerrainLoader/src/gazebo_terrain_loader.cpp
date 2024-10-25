#include "gazebo_terrain_loader.h"

namespace gazebo
{

  GazeboTerrainLoaderPlugin::GazeboTerrainLoaderPlugin() : WorldPlugin()
  {
    printf("Subscriber Plugin Created!\n");
    // model to use
    this->msg.set_sdf_filename("model://terrainLoaderBlock");
    this->transport_node = NULL;
  }

  GazeboTerrainLoaderPlugin::~GazeboTerrainLoaderPlugin()
  {
    this->gazebo_connection->~Connection();
  }

  void GazeboTerrainLoaderPlugin::Load(physics::WorldPtr _model, sdf::ElementPtr _sdf)
  {

    // using _model pointer & GetName()to get model name
    std::cout << "World Name = " << _model->Name() << std::endl;

    // set a node to subscribe
    this->transport_node = transport::NodePtr(new transport::Node());
    this->transport_node->Init();

    std::map<std::string, std::list<std::string>> topicsList = transport::getAdvertisedTopics();
    std::cout << "topic number" << topicsList.size() << std::endl;

    this->gazebo_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboTerrainLoaderPlugin::onEveryTick, this, _1));

    // set publisher
    this->publisher = this->transport_node->Advertise<msgs::Factory>("~/factory");
  }

  void GazeboTerrainLoaderPlugin::On_msg(ConstPosesStampedPtr &_msg)
  {

    // execute for every second
    time_t tempSeconds = time(NULL);

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

        // before spawning check whether the spawned block is already part of the already spawned block
        if (this->already_spawned_blocks_map.find({int_x, int_y}) == this->already_spawned_blocks_map.end())
        {
          this->bringUpStaticBlockOf1Meter(int_x, int_y, z);
          // remove from the file map such that memory will be saved
          this->file_data_map.erase({int_x, int_y});
        }
        else
        {
          if (tempSeconds > this->seconds)
          {
            this->seconds = tempSeconds;
            std::cout << "Already Spawned so not showing again" << std::endl;
          }
        }
      }
    }
  }

  void GazeboTerrainLoaderPlugin::bringUpStaticBlockOf1Meter(int &x_position, int &y_position, double &z_position)
  {
    // set model pose
    msgs::Set(this->msg.mutable_pose(), ignition::math::Pose3d(x_position, y_position, 0, 0, 0, 0));

    // Send the message
    this->publisher->Publish(this->msg);

    std::cout << "spawning the plane at position " << x_position << " - Xposition and " << y_position << " - Yposition!" << std::endl;

    // add the already spawned block here
    this->already_spawned_blocks_map[{x_position, y_position}] = z_position;
    std::cout << "Added the already spawned block with key " << x_position << "," << y_position << std::endl;
  }

  void GazeboTerrainLoaderPlugin::loadTheFileToMap()
  {
    std::ifstream file("/home/uav/Documents/terrainLoaderPlugin/trails/heightMask.csv");

    if (!file.is_open())
    {
      std::cerr << "Could not open the file!" << std::endl;
      return;
    }

    std::string line;
    // Skip the header line if it exists
    std::getline(file, line);

    while (std::getline(file, line))
    {
      std::istringstream ss(line);
      std::string x, y, z;

      if (std::getline(ss, x, ',') && std::getline(ss, y, ',') && std::getline(ss, z, ','))
      {
        int int_x = std::stoi(x);
        int int_y = std::stoi(y);
        double double_z = std::stod(z);
        this->file_data_map[{int_x, int_y}] = double_z; // Add to the map
      }
    }

    file.close();
  }

  void GazeboTerrainLoaderPlugin::onEveryTick(const common::UpdateInfo &_info)
  {
    // check if we stopped the every tick update execution
    if (this->stopOnEveryTickExecution)
    {
      return;
    }
    // as we not stopped execution lets keep on checking for the file
    // passing this section to see everything works fine as expected
    this->loadTheFileToMap();

    // on successful completion now subscribe to the topic
    // transport::SubscriberPtr subscribe
    this->sub = this->transport_node->Subscribe("~/pose/local/info", &GazeboTerrainLoaderPlugin::On_msg, this);
    this->stopOnEveryTickExecution = true;
    std::cout << "Setted the Plugin subscription from on every tick for now" << std::endl;
  }

  // Register plugin
  GZ_REGISTER_WORLD_PLUGIN(GazeboTerrainLoaderPlugin)
};
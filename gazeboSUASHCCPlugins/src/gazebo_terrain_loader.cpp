/*
 * Copyright 2024 Laxminarayana Vadnala, SLU, St. Louis, MO, USA
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "gazebo_terrain_loader.h"

namespace gazebo
{

  GazeboTerrainLoaderPlugin::GazeboTerrainLoaderPlugin() : WorldPlugin()
  {
    printf("Subscriber Plugin Created!\n");
    // model to use
    // this->msg.set_sdf_filename("model://terrainLoaderBlock");
    // std::cout << "loaded SDF file is :" << this->msg.sdf() << std::endl;
    this->terrainLoaderFilePath = DefaultFilePath;
    this->transport_node = NULL;
  }

  GazeboTerrainLoaderPlugin::~GazeboTerrainLoaderPlugin()
  {
    this->gazebo_connection->~Connection();
  }

  void GazeboTerrainLoaderPlugin::Load(physics::WorldPtr _model, sdf::ElementPtr _sdf)
  {
    this->world = _model;
    // using _model pointer & GetName()to get model name
    std::cout << "World Name = " << _model->Name() << std::endl;

    // set a node to subscribe
    this->transport_node = transport::NodePtr(new transport::Node());
    this->transport_node->Init();

    std::map<std::string, std::list<std::string>> topicsList = transport::getAdvertisedTopics();
    std::cout << "topic number" << topicsList.size() << std::endl;

    getSdfParam<std::string>(_sdf, "", terrainLoaderFilePath, terrainLoaderFilePath);

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
      // TODO: Ignore the ground_plane scenario
      if (msg_name.find("::") == std::string::npos)
      {
        double x = _msg->pose(i).position().x();
        double y = _msg->pose(i).position().y();
        double z = _msg->pose(i).position().z();
        double center_dertmination = 0;

        int int_x = (int)x;
        int int_y = (int)y;

        if (this->file_data_map.find({int_x, int_y}) != this->file_data_map.end()){
          double temp_z = this->file_data_map.at({int_x, int_y});
          if(temp_z<0){
            z = temp_z * -1;
            center_dertmination = temp_z/2;
          } else {
            z = temp_z;
            center_dertmination = temp_z/2;
          }
        }

        // before spawning check whether the spawned block is already part of the already spawned block
        if (this->already_spawned_blocks_map.find({int_x, int_y}) == this->already_spawned_blocks_map.end())
        {
          std::cout << "DRONE POSE SPAWNING IS " << int_x << "," << int_y << ":::::: " << z << " Center determiantion " << center_dertmination << std::endl;
          this->bringUpStaticBlockOf1Meter(int_x, int_y, z, center_dertmination);
          // remove from the file map such that memory will be saved
          this->file_data_map.erase({int_x, int_y});
        }
        // else
        // {
        //   if (tempSeconds > this->seconds)
        //   {
        //     this->seconds = tempSeconds;
        //     std::cout << "Already Spawned so not showing again" << std::endl;
        //   }
        // }
      }
    }
  }

  std::string GazeboTerrainLoaderPlugin::modifyXML(const std::string &xml, const std::string &newPose, const std::string &newSize)
  {
    // Regular expressions for matching the size and pose
    std::regex sizeRegex(R"(<size>\s*[^<]*\s*</size>)");
    std::regex poseRegex(R"(<pose>\s*[^<]*\s*</pose>)");

    // Replace size
    std::string modifiedXML = std::regex_replace(xml, sizeRegex, "<size>" + newSize + "</size>");

    // Replace pose (assuming pose tag exists, otherwise handle accordingly)
    modifiedXML = std::regex_replace(modifiedXML, poseRegex, "<pose>" + newPose + "</pose>");

    return modifiedXML;
  }

  void GazeboTerrainLoaderPlugin::bringUpStaticBlockOf1Meter(int &x_position, int &y_position, double &z_position, double &center_dertmination)
  {
    // set model pose
    // msgs::Set(this->msg.mutable_pose(), ignition::math::Pose3d(x_position, y_position, 0, 0, 0, 0));

    std::string newPose = std::to_string(x_position) + " " + std::to_string(y_position) + " " + std::to_string(center_dertmination) + " 0 0 0"; // Example new pose
    std::string newSize = "1 1 " + std::to_string(z_position);                                        // Example new size

    // // update the string to set the size and pose
    std::string temporaryModelSDF = this->modifyXML(this->xml, newPose, newSize);
    // this->modelSDF.Clear();
    // std::cout << " PRINTING XML >>>>>>>>>>>>>>>>>> " << temporaryModelSDF << std::endl;
    // this->modelSDF.SetFromString(temporaryModelSDF);
    // this->world->InsertModelSDF(this->modelSDF);
    this->world->InsertModelString(temporaryModelSDF);

    // Send the message
    // this->publisher->Publish(this->msg);

    // std::cout << "spawning the plane at position " << x_position << " - Xposition and " << y_position << " - Yposition!" << std::endl;

    // add the already spawned block here
    this->already_spawned_blocks_map[{x_position, y_position}] = true;
    // std::cout << "Added the already spawned block with key " << x_position << "," << y_position << std::endl;
  }

  void GazeboTerrainLoaderPlugin::loadTheFileToMap()
  {
    std::ifstream file(terrainLoaderFilePath);

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
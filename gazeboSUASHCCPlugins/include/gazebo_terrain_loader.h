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
#include <fstream>
#include <sstream>
#include <string>
#include <tuple>
#include <map>
#include <unordered_map>
#include <regex>
#include "common.h"

namespace gazebo
{

    // we are defining the constant for the file path which we can get from model file
    static const std::string DefaultFilePath = "/home/uav/Documents/terrainLoaderPlugin/trails/heightMask.csv";

    class GazeboTerrainLoaderPlugin : public WorldPlugin
    {
    public:
        GazeboTerrainLoaderPlugin();
        virtual ~GazeboTerrainLoaderPlugin();
        void Load(physics::WorldPtr _model, sdf::ElementPtr _sdf) override;

    protected:
        void onEveryTick(const common::UpdateInfo & /*_info*/);

    private:
        void On_msg(ConstPosesStampedPtr &_msg);
        void bringUpStaticBlockOf1Meter(int &x_position, int &y_position, double &z_position);
        void loadTheFileToMap();
        std::string modifyXML(const std::string& xml, const std::string& newPose, const std::string& newSize);

    private:
        // all XML config related value declaration
        std::string terrainLoaderFilePath;

        event::ConnectionPtr gazebo_connection;
        bool stopOnEveryTickExecution = false;
        transport::SubscriberPtr sub;
        msgs::Factory msg;
        sdf::SDF modelSDF;
        transport::NodePtr transport_node;
        physics::WorldPtr world;
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
        std::unordered_map<std::pair<int, int>, bool, PairHash> already_spawned_blocks_map;
        std::unordered_map<std::pair<int, int>, double, PairHash> file_data_map;

        std::string xml = R"(<sdf version='1.7'>
        <model name='plane'>
            <static>true</static>
            <pose>1 1 0 0 0 0</pose>
            <link name='link'>
                <collision name='collision'>
                    <geometry>
                        <box>
                            <size>1 1 1</size>
                        </box>
                    </geometry>
                </collision>
                <visual name='visual'>
                    <cast_shadows>false</cast_shadows>
                    <geometry>
                        <box>
                            <size>1 1 1</size>
                        </box>
                    </geometry>
                </visual>
            </link>
        </model>
    </sdf>)";
    };

} // namespace gazebo

#endif // GAZEBO_TERRAIN_LOADER_H

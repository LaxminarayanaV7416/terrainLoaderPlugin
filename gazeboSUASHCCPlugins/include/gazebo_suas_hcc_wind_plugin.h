/*
 * Copyright 2024 Bohan Zhang, SLU, St. Louis, MO, USA
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


#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H

#include <string>
#include <random>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
// #include "../include/data_processor/wind_data_processor.h"
#include "wind_data_processor.h"
#include "Wind.pb.h"

namespace gazebo {
// Default values
    static const std::string kDefaultNamespace = "";
    static const std::string kWindDataFile = "/home/uav/Documents/terrainLoaderPlugin/trails/wisp_50.csv";
    static const std::string kDefaultFrameId = "world";

    static constexpr double kDefaultWindVelocityMean = 0.0;
    static constexpr double kDefaultWindVelocityMax = 100.0;
    static constexpr double kDefaultWindVelocityVariance = 0.0;
    static constexpr double kDefaultWindGustVelocityMean = 0.0;
    static constexpr double kDefaultWindGustVelocityMax = 10.0;
    static constexpr double kDefaultWindGustVelocityVariance = 0.0;

    static constexpr double kDefaultWindGustStart = 10.0;
    static constexpr double kDefaultWindGustDuration = 0.0;

    static const ignition::math::Vector3d kDefaultWindDirectionMean = ignition::math::Vector3d(1, 0, 0);
    static const ignition::math::Vector3d kDefaultWindGustDirectionMean = ignition::math::Vector3d(0, 1, 0);
    static constexpr double kDefaultWindDirectionVariance = 0.0;
    static constexpr double kDefaultWindGustDirectionVariance = 0.0;

/// \brief This gazebo plugin simulates wind acting on a model.
    class GazeboWindPlugin : public WorldPlugin {
    public:
        GazeboWindPlugin()
                : WorldPlugin(),
                  namespace_(kDefaultNamespace),
                  wind_pub_topic_("world_wind"),
                  wind_velocity_mean_(kDefaultWindVelocityMean),
                  wind_velocity_max_(kDefaultWindVelocityMax),
                  wind_velocity_variance_(kDefaultWindVelocityVariance),
                  wind_gust_velocity_mean_(kDefaultWindGustVelocityMean),
                  wind_gust_velocity_max_(kDefaultWindGustVelocityMax),
                  wind_gust_velocity_variance_(kDefaultWindGustVelocityVariance),
                  wind_direction_mean_(kDefaultWindDirectionMean),
                  wind_gust_direction_mean_(kDefaultWindGustDirectionMean),
                  wind_direction_variance_(kDefaultWindDirectionVariance),
                  wind_gust_direction_variance_(kDefaultWindGustDirectionVariance),
                  frame_id_(kDefaultFrameId),
                  windDataFile(kWindDataFile),
                  pub_interval_(0.5),
                  node_handle_(NULL) {}

        virtual ~GazeboWindPlugin();

        WindDataProcessor::WindVal getNearestWindValue(float x, float y, float z);

    protected:
        /// \brief Load the plugin.
        /// \param[in] _model Pointer to the model that loaded this plugin.
        /// \param[in] _sdf SDF element that describes the plugin.
        void Load(physics::WorldPtr world, sdf::ElementPtr sdf);

        /// \brief Called when the world is updated.
        /// \param[in] _info Update timing information.
        void OnUpdate(const common::UpdateInfo& /*_info*/);

        /// \brief Retry loading the plugin if the model was not found initially.
        void RetryLoad();

        /// \brief Initialize the plugin after successful loading.
        void InitPlugin(sdf::ElementPtr sdf);

        bool LoadModels();
        bool LoadSingleModel(const std::string &modelName);
        bool LoadMultipleModels(const std::string &modelPrefix);



    private:
        /// \brief Pointer to the update event connection.

        std::string windDataFile;

        event::ConnectionPtr update_connection_;
        event::ConnectionPtr retry_load_connection_;
        std::unique_ptr<WindDataProcessor::WindDataProcessor> windProcessor;
        std::vector<physics::ModelPtr> models;

        physics::WorldPtr world_;

        std::string namespace_;

        std::string frame_id_;
        std::string wind_pub_topic_;

        double wind_velocity_mean_;
        double wind_velocity_max_;
        double wind_velocity_variance_;
        double wind_gust_velocity_mean_;
        double wind_gust_velocity_max_;
        double wind_gust_velocity_variance_;
        double pub_interval_;
        std::default_random_engine wind_velocity_generator_;
        std::normal_distribution<double> wind_velocity_distribution_;
        std::default_random_engine wind_gust_velocity_generator_;
        std::normal_distribution<double> wind_gust_velocity_distribution_;

        ignition::math::Vector3d wind_direction_mean_;
        ignition::math::Vector3d wind_gust_direction_mean_;
        double wind_direction_variance_;
        double wind_gust_direction_variance_;
        std::default_random_engine wind_direction_generator_;
        std::normal_distribution<double> wind_direction_distribution_X_;
        std::normal_distribution<double> wind_direction_distribution_Y_;
        std::normal_distribution<double> wind_direction_distribution_Z_;
        std::default_random_engine wind_gust_direction_generator_;
        std::normal_distribution<double> wind_gust_direction_distribution_X_;
        std::normal_distribution<double> wind_gust_direction_distribution_Y_;
        std::normal_distribution<double> wind_gust_direction_distribution_Z_;

        common::Time wind_gust_end_;
        common::Time wind_gust_start_;
        common::Time last_time_;

        transport::NodePtr node_handle_;
        transport::PublisherPtr wind_pub_;

        physics_msgs::msgs::Wind wind_msg;

        sdf::ElementPtr sdf_;  // Store the SDF pointer
    };
}

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H
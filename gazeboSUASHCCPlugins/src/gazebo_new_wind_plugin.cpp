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

#include "gazebo_new_wind_plugin.h"
#include "common.h"

namespace gazebo
{

    GazeboSuasHccWindPlugin::~GazeboSuasHccWindPlugin()
    {
        update_connection_->~Connection();
    }

    void GazeboSuasHccWindPlugin::loadCSVFile()
    {
        std::ifstream file(wind_data_csv_file_path);

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
        std::string x, y, z, u, v, w;
        std::getline(ss, x, ',');
        std::getline(ss, y, ',');
        std::getline(ss, z, ',');
        std::getline(ss, u, ',');
        std::getline(ss, v, ',');
        std::getline(ss, w, ',');
        //   std::cout << x << ", " << y << ", " << z << "::: "  << u << ", "  << v  << ", " << w << std::endl;
        if (!x.empty() && !y.empty() && !z.empty())
        {
            double double_x = roundToSixDecimals(std::stod(x));
            double double_y = roundToSixDecimals(std::stod(y));
            double double_z = roundToSixDecimals(std::stod(z));
            double double_u, double_v, double_w;
            if(!u.empty()){
                double_u = roundToSixDecimals(std::stod(u));
            } else {
                double_u = 0;
            }
            if(!v.empty()){
                double_v = roundToSixDecimals(std::stod(v));
            } else {
                double_v = 0;
            }
            if(!w.empty()){
                double_w = roundToSixDecimals(std::stod(w));
            } else {
                double_w = 0;
            }
            this->wind_file_hash_map[{double_x, double_y, double_z}] = {double_u, double_v, double_w}; // Add to the map
        }
        }
        file.close();
    }

    void GazeboSuasHccWindPlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf)
    {
        world_ = world;

        double wind_gust_start = kDefaultWindGustStart;
        double wind_gust_duration = kDefaultWindGustDuration;

        double wind_ramp_start = kDefaultWindRampStart;
        double wind_ramp_duration = kDefaultWindRampDuration;

        // if (sdf->HasElement("robotNamespace"))
        // {
        //     namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
        // }
        // else
        // {
        //     gzerr << "[gazebo_wind_plugin] Please specify a robotNamespace.\n";
        // }

        node_handle_ = transport::NodePtr(new transport::Node());
        node_handle_->Init(namespace_);

        // getting wind publishing topic name from SDF file we created
        getSdfParam<std::string>(sdf, "windPubTopic", wind_pub_topic_, wind_pub_topic_);
        double pub_rate = 2.0;
        getSdfParam<double>(sdf, "publishRate", pub_rate, pub_rate); // Wind topic publishing rates
        pub_interval_ = (pub_rate > 0.0) ? 1 / pub_rate : 0.0;
        getSdfParam<std::string>(sdf, "frameId", frame_id_, frame_id_);
        // Get the wind params from SDF.
        getSdfParam<double>(sdf, "windVelocityMean", wind_velocity_mean_, wind_velocity_mean_);
        getSdfParam<double>(sdf, "windVelocityMax", wind_velocity_max_, wind_velocity_max_);
        getSdfParam<double>(sdf, "windVelocityVariance", wind_velocity_variance_, wind_velocity_variance_);
        getSdfParam<ignition::math::Vector3d>(sdf, "windDirectionMean", wind_direction_mean_, wind_direction_mean_);
        getSdfParam<double>(sdf, "windDirectionVariance", wind_direction_variance_, wind_direction_variance_);
        // Get the wind gust params from SDF.
        getSdfParam<double>(sdf, "windGustStart", wind_gust_start, wind_gust_start);
        getSdfParam<double>(sdf, "windGustDuration", wind_gust_duration, wind_gust_duration);
        getSdfParam<double>(sdf, "windGustVelocityMean", wind_gust_velocity_mean_, wind_gust_velocity_mean_);
        getSdfParam<double>(sdf, "windGustVelocityMax", wind_gust_velocity_max_, wind_gust_velocity_max_);
        getSdfParam<double>(sdf, "windGustVelocityVariance", wind_gust_velocity_variance_, wind_gust_velocity_variance_);
        getSdfParam<ignition::math::Vector3d>(sdf, "windGustDirectionMean", wind_gust_direction_mean_, wind_gust_direction_mean_);
        getSdfParam<double>(sdf, "windGustDirectionVariance", wind_gust_direction_variance_, wind_gust_direction_variance_);

        // getting the file path to load the data file to the program
        getSdfParam<std::string>(sdf, "windDataFile", wind_data_csv_file_path, wind_data_csv_file_path);

        wind_direction_mean_.Normalize();
        wind_gust_direction_mean_.Normalize();
        wind_gust_start_ = common::Time(wind_gust_start);
        wind_gust_end_ = common::Time(wind_gust_start + wind_gust_duration);
        // Set random wind velocity mean and standard deviation
        wind_velocity_distribution_.param(std::normal_distribution<double>::param_type(wind_velocity_mean_, sqrt(wind_velocity_variance_)));
        // Set random wind direction mean and standard deviation
        wind_direction_distribution_X_.param(std::normal_distribution<double>::param_type(wind_direction_mean_.X(), sqrt(wind_direction_variance_)));
        wind_direction_distribution_Y_.param(std::normal_distribution<double>::param_type(wind_direction_mean_.Y(), sqrt(wind_direction_variance_)));
        wind_direction_distribution_Z_.param(std::normal_distribution<double>::param_type(wind_direction_mean_.Z(), sqrt(wind_direction_variance_)));
        // Set random wind gust velocity mean and standard deviation
        wind_gust_velocity_distribution_.param(std::normal_distribution<double>::param_type(wind_gust_velocity_mean_, sqrt(wind_gust_velocity_variance_)));
        // Set random wind gust direction mean and standard deviation
        wind_gust_direction_distribution_X_.param(std::normal_distribution<double>::param_type(wind_gust_direction_mean_.X(), sqrt(wind_gust_direction_variance_)));
        wind_gust_direction_distribution_Y_.param(std::normal_distribution<double>::param_type(wind_gust_direction_mean_.Y(), sqrt(wind_gust_direction_variance_)));
        wind_gust_direction_distribution_Z_.param(std::normal_distribution<double>::param_type(wind_gust_direction_mean_.Z(), sqrt(wind_gust_direction_variance_)));

        // Get the ramped wind params from SDF.
        getSdfParam<double>(sdf, "windRampStart", wind_ramp_start, wind_ramp_start);
        getSdfParam<double>(sdf, "windChangeRampDuration", wind_ramp_duration, wind_ramp_duration);
        getSdfParam<ignition::math::Vector3d>(sdf, "windRampWindVectorComponents", ramped_wind_vector, ramped_wind_vector);

        wind_ramp_start_ = common::Time(wind_ramp_start);
        wind_ramp_duration_ = common::Time(wind_ramp_duration);

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboSuasHccWindPlugin::OnUpdate, this, _1));

        this->wind_pub_ = node_handle_->Advertise<physics_msgs::msgs::Wind>("~/" + wind_pub_topic_); // here I have removed 10 am not sure what it is

        #if GAZEBO_MAJOR_VERSION >= 9
                last_time_ = world_->SimTime();
        #else
                last_time_ = world_->GetSimTime();
        #endif
    }

    void GazeboSuasHccWindPlugin::On_msg(ConstPosesStampedPtr &_msg)
    {
        #if GAZEBO_MAJOR_VERSION >= 9
            common::Time now = world_->SimTime();
        #else
            common::Time now = world_->GetSimTime();
        #endif

        for (int i = 0; i < _msg->pose_size(); i++)
        {
            std::string msg_name = _msg->pose(i).name();

            // this is the drone pose
            // TODO: Ignore the ground_plane scenario
            if (msg_name.find("::") == std::string::npos)
            {
                double x = this->roundToSixDecimals(_msg->pose(i).position().x());
                double y = this->roundToSixDecimals(_msg->pose(i).position().y());
                double z = this->roundToSixDecimals(_msg->pose(i).position().z());
                double u,v,w = 0;

                int int_x = (int)x;
                int int_y = (int)y;
                int int_z = (int)z;

                if(this->wind_file_hash_map.find({x, y, z})!= this->wind_file_hash_map.end()){
                    std::tuple<double, double, double> value = this->wind_file_hash_map.at({x, y, z});
                    u = std::get<0>(value);
                    v = std::get<1>(value);
                    w = std::get<2>(value);
                } else {
                    // key not found search in int dictionary
                    u = 10;
                    v = 0;
                    w = 0; 
                }

                gazebo::msgs::Vector3d* wind_v = new gazebo::msgs::Vector3d();
                wind_v->set_x(u);
                wind_v->set_y(v);
                wind_v->set_z(w);

                this->wind_msg.set_frame_id(this->frame_id_);
                this->wind_msg.set_time_usec(now.Double() * 1e6);
                this->wind_msg.set_allocated_velocity(wind_v);

                this->wind_pub_->Publish(this->wind_msg);

                std::cout << x << ", " << y << ", " << z << "::<" << u << ">, <" << v << ">, <" << w << ">" << std::endl;

            }
        }
    }

    // This gets called by the world update start event.
    void GazeboSuasHccWindPlugin::OnUpdate(const common::UpdateInfo &_info)
    {
        // check if we stopped the every tick update execution
        if (this->stopOnEveryTickExecution)
        {
            return;
        }
        // as we not stopped execution lets keep on checking for the file
        // passing this section to see everything works fine as expected
        this->loadCSVFile();

        // on successful completion now subscribe to the topic
        // transport::SubscriberPtr subscribe
        this->sub = this->node_handle_->Subscribe("~/pose/local/info", &GazeboSuasHccWindPlugin::On_msg, this);
        this->stopOnEveryTickExecution = true;
        std::cout << "Setted the Plugin subscription from on every tick for now" << std::endl;
    }

    GZ_REGISTER_WORLD_PLUGIN(GazeboSuasHccWindPlugin);
}
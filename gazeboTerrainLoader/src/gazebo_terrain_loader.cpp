#include <gazebo/gazebo.hh>                // for accessing all gazebo classes
#include <gazebo/common/common.hh>         // for common fn in gazebo like ModelPlugin
#include <gazebo/physics/physics.hh>       // for gazebo physics, to access -- ModelPtr

namespace gazebo {
    class SubscriberGazeboPlugin : public WorldPlugin {
      public:
        SubscriberGazeboPlugin() : WorldPlugin() {
            printf("Subscriber Plugin Created!\n");
        }

      public:
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
            
            // using _world pointer & GetName()to get model name
            std::cout << "World Name = " << _world->Name() << std::endl;

            // set a node to subscribe
            transport::NodePtr node(new transport::Node());
            node->Init();
            
            std::map<std::string, std::list<std::string>> topicsList = transport::getAdvertisedTopics();

            std::cout << "topic number" << topicsList.size() << std::endl;

            for (auto const& x : topicsList)
                {
                    std::cout << x.first  // string (key)
                            << ':' 
                            << std::endl;
                    std::cout << "------------------------------" << std::endl;

                    for (auto const& y: x.second) {
                        std::cout << y << std::endl; 
                    }

                    std::cout << "==============================" << std::endl;

                }

            // subscribe to topic 
            //transport::SubscriberPtr subscribe
            this->sub = node->Subscribe("~/pose/info", &SubscriberGazeboPlugin::On_msg, this);
        }
    
      public:
       void On_msg(ConstWorldStatisticsPtr &_msg)
        {
          // Dump the message contents to stdout.
          std::cout << _msg->DebugString();  //print output in the terminal
          
        }

     // good pratices to declare pub/sub as data member
     // reather than declaring them as local varibales which may cause issues
    private:
        transport::SubscriberPtr sub; 
    };
    // Register plugin 
    GZ_REGISTER_WORLD_PLUGIN(SubscriberGazeboPlugin)
}
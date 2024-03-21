#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {
    class MyGazeboPlugin : public WorldPlugin{
        public:
        MyGazeboPlugin() : WorldPlugin() {
            printf("Plugin constructor method!\n");
        }

        public:
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
            //set a node
            transport::NodePtr node(new transport::Node());
            node->Init(_world->Name());

            //set publisher
            transport::PublisherPtr publisher = node->Advertise<msgs::Factory>("~/factory");

            //create msg obj
            msgs::Factory msg;

            //model to use
            msg.set_sdf_filename("model://jersey_barrier");

            //send the message
            publisher->Publish(msg);
        }
    };

    //Register plugin
    GZ_REGISTER_WORLD_PLUGIN(MyGazeboPlugin)
}
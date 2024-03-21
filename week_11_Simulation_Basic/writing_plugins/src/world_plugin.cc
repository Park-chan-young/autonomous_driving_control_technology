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

            //setup new position
            msgs::Set(msg.mutable_pose(), ignition::math::Pose3d(ignition::math::Vector3d(5, 4, 0), ignition::math::Quaterniond(0, 0, 0)));

            //send the message
            publisher->Publish(msg);

        }
    };

    //resister plugin
    GZ_REGISTER_WORLD_PLUGIN(MyGazeboPlugin)
} // namespace gazebo
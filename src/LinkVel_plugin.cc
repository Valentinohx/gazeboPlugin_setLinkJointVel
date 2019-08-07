#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

#include "std_msgs/Float32MultiArray.h"

namespace gazebo
{
/// \brief A plugin to control a Velodyne sensor.
class LinkVelPlugin : public ModelPlugin
{
    /// \brief Constructor
public:
    LinkVelPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
public:
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // Safety check
        if (_model->GetJointCount() == 0)
        {
            std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
            return;
        }

        // Store the model pointer for convenience.
        this->model = _model;
        
      // Setup a P-controller, with a gain of 0.1.
      //this->pid = common::PID(0.1, 0, 0);

        // Get the first joint. We are making an assumption about the model
        // having one joint that is the rotational joint.
        
        double velocity[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};        
        this->SetVelocity(velocity);

        // Create the node
        this->node = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION < 8
        this->node->Init(this->model->GetWorld()->GetName());
#else
        this->node->Init(this->model->GetWorld()->Name());
#endif

// Create a topic name
        std::string topicName = "~/" + this->model->GetName() + "/LinkVel_cmd";

////////////////
// Initialize ros, if it has not already bee initialized.
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            //ros::init(argc, argv, "gazebo_client",
            ros::init(argc, argv, "gazebo",
                      ros::init_options::NoSigintHandler);
        }
// Create our ROS node. This acts in a similar manner to
// the Gazebo node
        this->rosNode.reset(new ros::NodeHandle("gazebo"));

// Create a named topic, and subscribe to it.
        ros::SubscribeOptions so =
            ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
                "/" + this->model->GetName() + "/LinkVel_cmd",
                1,
                boost::bind(&LinkVelPlugin::OnRosMsg, this, _1),
                ros::VoidPtr(), &this->rosQueue);
        this->rosSub = this->rosNode->subscribe(so);

// Spin up the queue helper thread.
        this->rosQueueThread =
            std::thread(std::bind(&LinkVelPlugin::QueueThread, this));
    }

public:
    void SetVelocity(const double _vel[])
    {
        this->model->GetLink("KUKA_LWR4P::fkuka_lwr4p_A6")->SetLinearVel({_vel[0], _vel[1], _vel[2]});
        this->model->GetLink("KUKA_LWR4P::fkuka_lwr4p_A6")->SetAngularVel({_vel[3], _vel[4], _vel[5]});
    }

/// \brief Handle an incoming message from ROS
/// \param[in] _msg A float value that is used to set the velocity
/// of the Velodyne.
public:
    void OnRosMsg(const std_msgs::Float32MultiArrayConstPtr &_msg)
    {
        
        for (int i = 0; i<6; i++)
           _vel_data[i] = _msg->data[i];
        this->SetVelocity(_vel_data);
    }

/// \brief ROS helper function that processes messages
private:
    void QueueThread()
    {
        static const double timeout = 0.01;
        while (this->rosNode->ok())
        {
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
        }
    }

    /// \brief Pointer to the model.
private:
    physics::ModelPtr model;

/// \brief Pointer to the joint.
private:
    physics::JointPtr joint;

/// \brief A PID controller for the joint.
private:
    common::PID pid;

/// \brief A node used for transport
private:
    transport::NodePtr node;

/// \brief A subscriber to a named topic.
private:
    transport::SubscriberPtr sub;

    /// \brief A node use for ROS transport
private:
    std::unique_ptr<ros::NodeHandle> rosNode;

/// \brief A ROS subscriber
private:
    ros::Subscriber rosSub;

/// \brief A ROS callbackqueue that helps process messages
private:
    ros::CallbackQueue rosQueue;

/// \brief A thread the keeps running the rosQueue
private:
    std::thread rosQueueThread;

private:
    double _vel_data[6];
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(LinkVelPlugin)
}
#endif

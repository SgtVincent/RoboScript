#ifndef GAZEBO_ROS_JOINT_AXIS_SERVICE_H
#define GAZEBO_ROS_JOINT_AXIS_SERVICE_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <gazebo/gazebo.hh>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/common/common.hh>
#include <gazebo_plugins/gazebo_ros_joint_axis_service.h>
#include <joint_prediction/GazeboGetJointsAxes.h>

namespace gazebo
{
    
    class GazeboRosJointAxisService : public gazebo::ModelPlugin
    {
    public:
        GazeboRosJointAxisService();
        virtual ~GazeboRosJointAxisService();

        // Overridden Gazebo entry points
        virtual void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

    private:
        bool getJointAxisCallback(joint_prediction::GazeboGetJointsAxes::Request &req, joint_prediction::GazeboGetJointsAxes::Response &res);

        // Pointer to the gazebo world.
        private: physics::WorldPtr world_;
        // Pointer to the model
        gazebo::physics::ModelPtr model_;
        // Node name
        std::string node_name_;
        // ROS Nodehandle
        ros::NodeHandlePtr nh_;        
        // ROS Spinner 
        ros::AsyncSpinner spinner_;
        // Callback queue for processing service callback functions
        ros::CallbackQueue callback_queue_;
        // ROS Service
        ros::ServiceServer joint_axis_service_;
    };
} // namespace gazebo

#endif // GAZEBO_ROS_JOINT_AXIS_SERVICE_H
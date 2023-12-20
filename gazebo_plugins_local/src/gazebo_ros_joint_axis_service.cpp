/*
 * Description: A plugin to host a ROS service to get joint axis information
 * Junting Chen, Dec. 2023
 */
#include "gazebo_ros_joint_axis_service.h"

namespace gazebo {
    
    GZ_REGISTER_MODEL_PLUGIN(GazeboRosJointAxisService);

    ////////////////////////////////////////////////////////////////////////////////
    // Constructor
    GazeboRosJointAxisService::GazeboRosJointAxisService()
        : spinner_(1, &callback_queue_) {}

    ////////////////////////////////////////////////////////////////////////////////
    // Destructor
    GazeboRosJointAxisService::~GazeboRosJointAxisService()
    {
        this->nh_->shutdown();
    }

    void gazebo::GazeboRosJointAxisService::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {   
        // Get world from gazebo
        this->world_ = _model->GetWorld();

        // Get name of this plugin 
        this->node_name_ = "GazeboRosJointAxisPlugin"; // default name
        if (_sdf->HasElement("NodeName"))
        {
            this->node_name_ = _sdf->GetElement("NodeName")->Get<std::string>();
        }

        this->model_ = _model;
        this->nh_.reset(new ros::NodeHandle(this->node_name_));
        this->nh_->setCallbackQueue(&callback_queue_);
        this->spinner_.start();

        this->joint_axis_service_ = this->nh_->advertiseService("/gazebo/get_joints_axes", &GazeboRosJointAxisService::getJointAxisCallback, this);
    }

    bool gazebo::GazeboRosJointAxisService::getJointAxisCallback(gazebo_plugins_local::GazeboGetJointsAxes::Request &req, gazebo_plugins_local::GazeboGetJointsAxes::Response &res)
    {
        for (const std_msgs::String& joint_name : req.joint_names)
        {
            // Create a new Joint Axis message
            joint_prediction::JointAxis joint_axis;

            // convert to std_msgs::String to std::string
            std::string full_joint_name_str = joint_name.data;
            joint_axis.joint_name.data = full_joint_name_str;

            // The joint has name format <model_name>::<joint_name>, e.g. cabinet::joint_0
            // First get the model handle and then the joint handle
            std::string model_name = full_joint_name_str.substr(0, full_joint_name_str.find("::"));
            std::string joint_name_str = full_joint_name_str.substr(full_joint_name_str.find("::") + 2, full_joint_name_str.length());
            
            // Get model from world
            gazebo::physics::ModelPtr model = this->world_->ModelByName(model_name);
            if (!model) {
                ROS_ERROR("Model %s not found, returning zero axis", model_name.c_str());
                joint_axis.axis.x = 0.0;
                joint_axis.axis.y = 0.0;
                joint_axis.axis.z = 0.0;
                res.joints_axes.push_back(joint_axis);
                continue;
            } 
            
            // Get joint from model 
            gazebo::physics::JointPtr joint = model->GetJoint(joint_name_str);
            if (!joint)
            {
                ROS_ERROR("Joint %s of model %s not found, returning zero axis", joint_name_str.c_str(), model_name.c_str());
                joint_axis.axis.x = 0.0;
                joint_axis.axis.y = 0.0;
                joint_axis.axis.z = 0.0;
                res.joints_axes.push_back(joint_axis);
                continue;

            }

            // Get joint axis vector 
            ignition::math::Vector3d global_axis = joint->GlobalAxis(0).Normalized();
            joint_axis.axis.x = global_axis.X();
            joint_axis.axis.y = global_axis.Y();
            joint_axis.axis.z = global_axis.Z();

            // Get joint type 
            auto joint_type = joint->GetMsgType();
            joint_axis.type = static_cast<uint8_t>(joint_type);
            
            // Get joint origin vector: 
            // Joint_Type_REVOLUTE = 1, use Joint Anchor position as origin
            // Joint_Type_PRISMATIC = 3, use its child link's center as origin
            if (joint_axis.type == 1)
            {
                ignition::math::Vector3d joint_origin = joint->Anchor(0);
                joint_axis.origin.x = joint_origin.X();
                joint_axis.origin.y = joint_origin.Y();
                joint_axis.origin.z = joint_origin.Z();
            }
            else if (joint_axis.type == 3)
            {
                ignition::math::Vector3d joint_origin = joint->GetChild()->BoundingBox().Center();
                joint_axis.origin.x = joint_origin.X();
                joint_axis.origin.y = joint_origin.Y();
                joint_axis.origin.z = joint_origin.Z();
            }
            else
            {
                ROS_ERROR("Joint type %d not supported, returning zero origin", joint_axis.type);
                joint_axis.origin.x = 0.0;
                joint_axis.origin.y = 0.0;
                joint_axis.origin.z = 0.0;
            }
            
            res.joints_axes.push_back(joint_axis);
        }

        return true;
    }


}

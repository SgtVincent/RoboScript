#include <gazebo_plugins/gazebo_ros_joint_axis_service.h>

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

    bool gazebo::GazeboRosJointAxisService::getJointAxisCallback(joint_prediction::GazeboGetJointsAxes::Request &req, joint_prediction::GazeboGetJointsAxes::Response &res)
    {
        for (const std_msgs::String& joint_name : req.joint_names)
        {
            // Create a new vector3 message to store the axis
            geometry_msgs::Vector3 axis;

            // convert to std_msgs::String to std::string
            std::string full_joint_name_str = joint_name.data;

            // The joint has name format <model_name>::<joint_name>, e.g. cabinet::joint_0
            // First get the model handle and then the joint handle
        
            std::string model_name = full_joint_name_str.substr(0, full_joint_name_str.find("::"));
            std::string joint_name_str = full_joint_name_str.substr(full_joint_name_str.find("::") + 2, full_joint_name_str.length());
            
            // Get model from world
            gazebo::physics::ModelPtr model = this->world_->ModelByName(model_name);
            if (!model)
            {
                ROS_ERROR("Model %s not found, returning zero axis", model_name.c_str());
                axis.x = 0.0;
                axis.y = 0.0;
                axis.z = 0.0;
            } else {
                // Get joint from model 
                gazebo::physics::JointPtr joint = model->GetJoint(joint_name_str);
                if (!joint)
                {
                    ROS_ERROR("Joint %s of model %s not found, returning zero axis", joint_name_str.c_str(), model_name.c_str());
                    axis.x = 0.0;
                    axis.y = 0.0;
                    axis.z = 0.0;
                } else {
                    ignition::math::Vector3d global_axis = joint->GlobalAxis(0).Normalized();
                    axis.x = global_axis.X();
                    axis.y = global_axis.Y();
                    axis.z = global_axis.Z();
                } 
            }

            res.joints_axes.push_back(axis);
        }

        return true;
    }


}

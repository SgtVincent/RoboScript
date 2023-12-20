/*
 * Description: A plugin to provide models and furniture parts (if any) bounding box information as a ROS service
 * Junting Chen, Oct 2023
 */

#include "gazebo_ros_bbox_3d_plugin.h"

namespace gazebo
{

    GZ_REGISTER_MODEL_PLUGIN(BoundingBoxPlugin);

    ////////////////////////////////////////////////////////////////////////////////
    // Constructor
    BoundingBoxPlugin::BoundingBoxPlugin() 
        : spinner_(1, &callback_queue_) {}

    ////////////////////////////////////////////////////////////////////////////////
    // Destructor
    BoundingBoxPlugin::~BoundingBoxPlugin()
    {
        this->rosnode_->shutdown();
    }

    void BoundingBoxPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        this->world_ = _model->GetWorld();
        this->model_name_ = _model->GetName();

        /* loop to wait for vscode ROS attach debugger
        // wait for debugger
        int flag = 0;
        while (flag == 0) {
          ROS_INFO_NAMED("BoundingBoxPlugin", "waiting for debugger");
          sleep(1);
        }
        */
        // extra links to publish bounding box
        this->links_to_publish_ = std::vector<std::string>();
        if (!_sdf->HasElement("links_to_publish"))
        {
            ROS_INFO_NAMED("BoundingBoxPlugin", "missing <links_to_publish>, defaults to no links");
        }
        else
        {
            // read a vector of strings from links_to_publish sdf element
            sdf::ElementPtr sdf_links_to_publish = _sdf->GetElement("links_to_publish");
            // read all child elements
            sdf::ElementPtr sdf_link = sdf_links_to_publish->GetFirstElement();
            while (sdf_link)
            {
                // add link name to vector
                this->links_to_publish_.push_back(sdf_link->Get<std::string>());
                // get next child element
                sdf_link = sdf_link->GetNextElement();
            }
            ROS_INFO_NAMED("BoundingBoxPlugin", "publishing bounding box for links: %s",
                           boost::algorithm::join(this->links_to_publish_, ", ").c_str());
        }

        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                             << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        this->rosnode_.reset(new ros::NodeHandle(this->robot_namespace_));
        this->rosnode_->setCallbackQueue(&callback_queue_);
        this->spinner_.start();
        
        // Create a service to handle requests for bounding box information
        get_bboxes_3d_service_ = this->rosnode_->advertiseService("/gazebo/get_bounding_boxes",
                                                                       &BoundingBoxPlugin::GetBoundingBoxes, this);
    }

    bool BoundingBoxPlugin::GetBoundingBoxes(gazebo_plugins_local::GazeboGetBoundingBoxes::Request &req,
                                            gazebo_plugins_local::GazeboGetBoundingBoxes::Response &res)
    {
        // Iterate over each model in the world
        for (const auto &model : world_->Models())
        {
            // Get the model's pose
            const auto &modelName = model->GetName();
            const auto &pose = model->WorldPose();

            // Get the model's axis-aligned bounding box center and size
            const ignition::math::AxisAlignedBox &bbox = model->BoundingBox();
            const ignition::math::Vector3d &center = bbox.Center();
            const ignition::math::Vector3d &size = bbox.Size();

            // Create a ROS message to publish the bounding box information
            grasp_detection::BoundingBox3D bbox3d;
            bbox3d.object_id = modelName;
            bbox3d.center.position.x = center.X();
            bbox3d.center.position.y = center.Y();
            bbox3d.center.position.z = center.Z();
            bbox3d.size.x = size.X();
            bbox3d.size.y = size.Y();
            bbox3d.size.z = size.Z();

            // Add the 3D bounding box to the array
            res.bboxes_3d.push_back(bbox3d);
        }

        // Also add extra links to the detection array if they are in the world
        for (const auto &linkName : links_to_publish_)
        {
            // Get the link
            const auto &link = world_->EntityByName(linkName);
            if (link == nullptr)
            {
                ROS_WARN_STREAM("Link " << linkName << " does not exist in the world");
                continue;
            }

            // Get the link's pose
            const auto &pose = link->WorldPose();

            // Get the link's axis-aligned bounding box center and size
            const ignition::math::AxisAlignedBox &bbox = link->BoundingBox();
            const ignition::math::Vector3d &center = bbox.Center();
            const ignition::math::Vector3d &size = bbox.Size();

            // Create a ROS message to publish the bounding box information
            grasp_detection::BoundingBox3D bbox3d;
            bbox3d.object_id = linkName;
            bbox3d.center.position.x = center.X();
            bbox3d.center.position.y = center.Y();
            bbox3d.center.position.z = center.Z();
            bbox3d.size.x = size.X();
            bbox3d.size.y = size.Y();
            bbox3d.size.z = size.Z();

            // Add the 3D bounding box to the array
            res.bboxes_3d.push_back(bbox3d);
        }

        return true;
    }

} // namespace gazebo

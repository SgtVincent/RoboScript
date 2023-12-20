#ifndef GAZEBO_ROS_BBOX_3D_PLUGIN_H
#define GAZEBO_ROS_BBOX_3D_PLUGIN_H

#include <string>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <ros/callback_queue.h>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>

#include <ignition/math/AxisAlignedBox.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/transport/Node.hh>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <grasp_detection/BoundingBox3DArray.h>
#include <grasp_detection/BoundingBox3D.h>

#include "gazebo_plugins_local/GazeboGetBoundingBoxes.h"

namespace gazebo
{
    /// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
    /// @{
    /** \defgroup GazeboRosBoundingBoxPlugin XML Reference and Example

      \brief Plugin to publish ground truth bounding boxes loaded in gazebo.

      This plugin publishes the ground truth bounding boxes loaded in gazebo as a
      vision_msgs/Detection3DArray message. The plugin is loaded in the robot definition xacro file.

      Example Usage:

      \verbatim
      <gazebo>
        <plugin name="gazebo_ros_bbox_3d_plugin" filename="libgazebo_ros_bbox_3d_plugin.so">
          <updatePeriod>1.0</updatePeriod>
        </plugin>
      </gazebo>
      \endverbatim
    \{
    */

    class BoundingBoxPlugin : public ModelPlugin
    {
        /// \brief Constructor
    public:
        BoundingBoxPlugin();

        /// \brief Destructor
    public:
        virtual ~BoundingBoxPlugin();

        // Documentation inherited
    protected:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    private:
        bool GetBoundingBoxes(gazebo_plugins_local::GazeboGetBoundingBoxes::Request &req,
                                            gazebo_plugins_local::GazeboGetBoundingBoxes::Response &res);
    private:
        physics::WorldPtr world_;
        /// \brief A pointer to the Model of the robot doing the planning
        physics::ModelPtr model_;

        /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
    private:
        boost::scoped_ptr<ros::NodeHandle> rosnode_;
        // ROS Spinner
        ros::AsyncSpinner spinner_;
        // Callback queue for processing service callback functions
        ros::CallbackQueue callback_queue_;
        ros::ServiceServer get_bboxes_3d_service_;

    private:
        std::string topic_name_;
        /// \brief The MoveIt scene name
        std::string scene_name_;
        std::string robot_name_;
        std::string model_name_;
        std::string robot_namespace_;
        std::vector<std::string> links_to_publish_;
    };
    /** \} */
    /// @}
}
#endif
#ifndef GAZEBO_ROS_BBOX_3D_PLUGIN_H
#define GAZEBO_ROS_BBOX_3D_PLUGIN_H

#include <string>
#include <geometry_msgs/Pose.h>

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <ignition/math/AxisAlignedBox.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/transport/Node.hh>
#include <std_srvs/Empty.h>

#include <grasp_detection/BoundingBox3DArray.h>
#include <grasp_detection/BoundingBox3D.h>


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
  public: BoundingBoxPlugin();

  /// \brief Destructor
  public: virtual ~BoundingBoxPlugin();

  // Documentation inherited
  protected: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  // Documentation inherited
  protected: virtual void UpdateCB();

  protected: void subscriber_connected();

  /// \brief A pointer to the gazebo world.
  private: physics::WorldPtr world_;

  /// \brief A pointer to the Model of the robot doing the planning
  private: physics::ModelPtr model_;

  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  private: boost::scoped_ptr<ros::NodeHandle> rosnode_;

  /// \brief A mutex to lock access to fields that are used in ROS message callbacks
  // private: boost::mutex mutex_; // no racing condition

  /// \brief ROS topic name inputs
  private: std::string topic_name_;
  /// \brief The MoveIt scene name
  private: std::string scene_name_;
  private: std::string robot_name_;
  private: std::string model_name_;

  /// \brief for setting ROS name space
  private: std::string robot_namespace_;

  // Pointer to the update event connection
  private: event::ConnectionPtr update_connection_;
  private: event::ConnectionPtr add_connection_;
  private: event::ConnectionPtr delete_connection_;

  private: ros::Publisher bboxes_3d_pub_;

  private: ros::Duration publish_period_;
           std::vector<std::string> links_to_publish_;
           ros::Time last_publish_time_;
  
};
/** \} */
/// @}
}
#endif
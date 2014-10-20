#ifndef KR_VIZ_RVIZ_HELPER_TF_PUBLISHER_HPP_
#define KR_VIZ_RVIZ_HELPER_TF_PUBLISHER_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

namespace kr {
namespace viz {

class TfPublisher {
 public:
  TfPublisher() = default;
  TfPublisher(const std::string &child_frame_id)
      : child_frame_id_(child_frame_id) {}

  const std::string &child_frame_id() const { return child_frame_id_; }
  void set_child_frame_id(const std::string &id) { child_frame_id_ = id; }

  /**
   * @brief PublishTransform
   */
  void PublishTransform(const geometry_msgs::Pose &pose,
                        const std_msgs::Header &header) {
    geometry_msgs::Vector3 translation;
    translation.x = pose.position.x;
    translation.y = pose.position.y;
    translation.z = pose.position.z;

    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header = header;
    transform_stamped.child_frame_id = child_frame_id_;
    transform_stamped.transform.translation = translation;
    transform_stamped.transform.rotation = pose.orientation;

    Publish(transform_stamped);
  }

 private:
  void Publish(const geometry_msgs::TransformStamped &tf_stamped) {
    tf_broadcaster_.sendTransform(tf_stamped);
  }

  std::string child_frame_id_;                    ///< Child frame id
  tf2_ros::TransformBroadcaster tf_broadcaster_;  ///< Tf broadcaster
};

}  // namespace viz
}  // namespace kr

#endif  // KR_VIZ_RVIZ_HELPER_TF_PUBLISHER_HPP_

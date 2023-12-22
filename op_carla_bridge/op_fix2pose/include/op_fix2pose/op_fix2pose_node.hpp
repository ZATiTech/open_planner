
#ifndef OP_FIX2POSE_
#define OP_FIX2POSE_

#include <string>
#include <vector>
#include <ostream>
#include <algorithm>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class Fix2PoseInterface : public rclcpp::Node
{
public:
   explicit Fix2PoseInterface(const rclcpp::NodeOptions & node_options);
   virtual ~Fix2PoseInterface();
   std::string tf_output_frame_;
  
private:

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gnss_fix;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pup_pose;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pup_pose_with_cov;


  void GnssCallBack(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

};

#endif  // OP_FIX2POSE_

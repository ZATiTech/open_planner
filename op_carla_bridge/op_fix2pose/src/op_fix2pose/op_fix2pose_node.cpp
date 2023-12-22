
#include "op_fix2pose/op_fix2pose_node.hpp"
#include <yaml-cpp/yaml.h>
#include <mapping/MappingHelpers.h>

void Fix2PoseInterface::GnssCallBack(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
	
	
	// std::cout << "Received GNSS scan : " << msg->longitude << ", " << msg->latitude << std::endl;

	geometry_msgs::msg::PoseStamped pose_;
	geometry_msgs::msg::PoseWithCovarianceStamped pose_with_cov_;

	op::WayPoint org, p;

	op::mapping::MappingHelpers::llaToxyz_proj("+proj=tmerc +lat_0=0 +lon_0=0 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs", org, msg->latitude, msg->longitude, msg->altitude, p.pos.x, p.pos.y, p.pos.z);	

	// std::cout << "Local Coord Pose: " << p.pos.x << ", " <<  p.pos.y << ", " <<  p.pos.z << std::endl;

	pose_.header = msg->header;
	pose_.pose.position.x = p.pos.x;
	pose_.pose.position.y = p.pos.y;
	pose_.pose.position.z = p.pos.z;

	pose_with_cov_.header = pose_.header;
	pose_with_cov_.pose.pose = pose_.pose;

	pup_pose->publish(pose_);
	pup_pose_with_cov->publish(pose_with_cov_);

}

Fix2PoseInterface::~Fix2PoseInterface()
{

}

Fix2PoseInterface::Fix2PoseInterface(const rclcpp::NodeOptions & node_options)
: Node("op_fix2pose_node", node_options), tf_output_frame_("gnss_link")
{
	sub_gnss_fix = this->create_subscription<sensor_msgs::msg::NavSatFix>(
		    "carla_nav_sat_fix", 1,
		    std::bind(&Fix2PoseInterface::GnssCallBack, this, std::placeholders::_1));

	pup_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>(
		"/sensing/gnss/pose", 1);
	pup_pose_with_cov = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
		"/sensing/gnss/pose_with_covariance", 1);	  
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Fix2PoseInterface)

#ifndef ROS2__SYSTEMCTLCONTROLLER
#define ROS2__SYSTEMCTLCONTROLLER

#include <rclcpp/rclcpp.hpp>

namespace addons {

	class SystemctlController : public rclcpp::Node {
		public:
				
			SystemctlController(const rclcpp::NodeOptions & options);
			
			~SystemctlController(){};

			void start_srv(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
									  	    std::shared_ptr<std_srvs::srv::Empty::Response> response); 

			void stop_srv(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
							          std::shared_ptr<std_srvs::srv::Empty::Response> response);		

			void query_status(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
							              std::shared_ptr<std_srvs::srv::Empty::Response> response);
 
  };
} // namespace addons


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(addons::SystemctlController)

#endif //ROS2__CLOUD_MUXER
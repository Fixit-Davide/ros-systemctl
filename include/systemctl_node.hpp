#ifndef ROS2__SYSTEMCTLNODE
#define ROS2__SYSTEMCTLNODE

#include <rclcpp/rclcpp.hpp>
#include "std_srvs/srv/empty.hpp"


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

			void restart_srv(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
							              std::shared_ptr<std_srvs::srv::Empty::Response> response);
        
    private:
    
      std::string start_service_name_;
      std::string stop_service_name_;
      std::string restart_service_name_;
      std::string query_service_name_;
 
      rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_srv_;
      rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv_;
      rclcpp::Service<std_srvs::srv::Empty>::SharedPtr restart_srv_;
      rclcpp::Service<std_srvs::srv::Empty>::SharedPtr query_srv_;
  };
} // namespace addons


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(addons::SystemctlController)

#endif //ROS2__SYSTEMCTLNODE
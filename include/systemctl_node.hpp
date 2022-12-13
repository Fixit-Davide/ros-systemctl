#ifndef ROS2__SYSTEMCTLNODE
#define ROS2__SYSTEMCTLNODE

#include <rclcpp/rclcpp.hpp>
#include "std_srvs/srv/trigger.hpp"


namespace addons {

	class SystemctlController : public rclcpp::Node {
		public:
			
			SystemctlController(const rclcpp::NodeOptions & options);
			
			~SystemctlController(){};

			void service_call(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> response,
                                        const std::string &method,
                                        std::string sys_ser); 

			void query_status(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response,
                        const std::string &property,
                        std::string sys_res);

    private:

      std::string start_service_name_;
      std::string stop_service_name_;
      std::string restart_service_name_;
      std::string query_service_name_;
      std::vector<std::string> sys_services_list;

      std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> start_srvs_;
      std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> stop_srvs_;
      std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> restart_srvs_;
      std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> query_srvs_;
  };
} // namespace addons


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(addons::SystemctlController)

#endif //ROS2__SYSTEMCTLNODE
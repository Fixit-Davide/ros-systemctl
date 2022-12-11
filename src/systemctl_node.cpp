#include <rclcpp/rclcpp.hpp>
#include <systemd/sd-bus.h>
//#include "ros2_srv/srv/Callersrv.hpp"
#include <std_srvs/srv/empty.hpp>
#include "systemctl_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace addons{

  SystemctlController::SystemctlController(const rclcpp::NodeOptions & options) 
  : Node("systemctl_controller", options) {

    start_service_name_ = declare_parameter<std::string>("services.start", "");
    stop_service_name_ = declare_parameter<std::string>("services.stop", "");

    start_srv_ = create_service<std_srvs::srv::Empty>(start_service_name_, std::bind(&SystemController::start_srv, this, _1, _2));
    stop_srv_ = create_service<std_srvs::srv::Empty>(stop_service_name_, std::bind(&SystemController::stop_srv, this, _1, _2));   
  }

  SystemctlController::~SystemctlController()
  {
    // free the Bus when the destructor is called
    sd_bus_unref(bus);
  }

  void SystemctlController::start_srv(const std::shared_ptr<std_srvs::srv::Empty::Request>
                                    std::shared_ptr<std_srvs::srv::Empty::Response> ) {
    // open the systemd bus
    sd_bus *bus = NULL;
    sd_bus_error *err = SD_BUS_ERROR_NULL;

    int ret = sd_bus_open_system(&bus);
    // analyze the sd_bus_default_system
    if (ret < 0) {
	    RCLCPP_INFO(get_logger(), "Could not open the Bus. Return: %d ", ret);     
      return;
    }

    const char *service = "my_service";
    const char *interface = "org.freedesktop.systemd1.Service";
    const char *method = "Start";
    ret = sd_bus_call_method(bus, service, interface, method, err, NULL);
    if (ret < 0) {
	    RCLCPP_INFO(get_logger(), "Could not Start the Service. Return: %d ", ret);
    }
      // Clean up
    sd_bus_unref(bus);
    sd_bus_error_free(err);
  }

  void SystemctlController::stop_srv(const std::shared_ptr<std_srvs::srv::Empty::Request>
                                    std::shared_ptr<std_srvs::srv::Empty::Response> ) {
    // open the systemd bus
    sd_bus *bus = NULL;
    sd_bus_error *err = SD_BUS_ERROR_NULL;
   
    int ret = sd_bus_open_system(&bus);
    if (ret < 0) {
      RCLCPP_INFO(get_logger(), "Could not open the Bus. Return: %d ", ret);     
      return;
    }

    const char *service = "my_service";
    const char *interface = "org.freedesktop.systemd1.Service";
    const char *method = "Stop";
    ret = sd_bus_call_method(bus, service, interface, method, err, NULL);
    if (ret < 0) {
	    RCLCPP_INFO(get_logger(), "Could not Stop the Service. Return: %d ", ret);
    }
    sd_bus_unref(bus);
    sd_bus_error_free(err);
  }

  void SystemctlController::query_status(const std::shared_ptr<std_srvs::srv::Empty::Request>
                                      std::shared_ptr<std_srvs::srv::Empty::Response> ) {
    // open the systemd bus
    sd_bus *bus = NULL;
    sd_bus_error *err = SD_BUS_ERROR_NULL;
    int ret = sd_bus_open_system(&bus);

    // analyze the sd_bus_default_system
    if (ret < 0) {
	    RCLCPP_INFO(get_logger(), "Could not open the Bus. Return: %d ", ret);     
      return;
    }

    const char *service = "my_service";
    const char *interface = "org.freedesktop.systemd1.Service";
    const char *method = "ActiveState";
    ret = sd_bus_get_property_string(bus, service, interface, method, err, NULL);
    if (ret < 0) {
      std::string err_msg(err.message);
      RCLCPP_INFO(get_logger(), "Failed to get "+method+" for service "+service+". Error: "+err_msg);
      // Clean error.
      sd_bus_error_free(err);
    }
    // Clean bus.
    sd_bus_unref(bus);
  }
}


/*
class SystemctlController : public rclcpp::Node {
    public:
        SystemctlController() : Node("systemctl_controller") {

            subscription_ = this->create_subscription<std_msgs::msg::String>(
                "systemctl_commands",
                10,
                [this](const std_msgs::msg::String::SharedPtr msg){
                    this->command_callback(msg);
                }
            );
        }
}

private:
*/
/*
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

class SystemctlControl : public rclcpp::Node
{
public:
  SystemctlControl() : Node("systemctl_control")
  {
    // Create the service client for calling the `systemctl` command
    systemctl_client_ = this->create_client<std_srvs::srv::Empty>("/systemctl");

    // Create a timer to periodically check the status of the service
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&SystemctlControl::check_service_status, this));
  }

private:
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr systemctl_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  // This function is called periodically to check the status of the service
  void check_service_status()
  {
    // Check if the service is available
    if (!systemctl_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(this->get_logger(), "Service not available");
      return;
    }

    // Create a request message
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();

    // Call the service to start the service
    auto result = systemctl_client_->async_send_request(request);

    // Check the result of the service call
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Service call failed");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Service started successfully");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  disl::SystemctlControl systemctl_control;
  rclcpp::spin(systemctl_control.get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}

*/

/*
sd_bus *bus = NULL;
int ret = sd_bus_open_system(&bus);
if (ret < 0) {
    // handle error
}
const char *service = "my_service";
const char *interface = "org.freedesktop.systemd1.Service";
const char *method = "Start";

ret = sd_bus_call_method(bus, service, interface, method, NULL, NULL);
if (ret < 0) {
    // handle error
}

const char *method = "Stop";

ret = sd_bus_call_method(bus, service, interface, method, NULL, NULL);
if (ret < 0) {
    
*/

/*
#include <sd-bus.h>
#include <ros2/ros2.hpp>

int main(int argc, char **argv)
{
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Connect to the sd-bus system bus
  sd_bus *bus = NULL;
  int ret = sd_bus_open_system(&bus);
  if (ret < 0) {
    // Handle error
  }

  // Create a sd-bus message
  sd_bus_message *m = NULL;
  ret = sd_bus_message_new_method_call(bus,
                                       &m,
                                       "org.example.service",  // Service name
                                       "/org/example/service", // Service object path
                                       "org.example.service.interface", // Service interface
                                       "MethodName"); // Method to call
  if (ret < 0) {
    // Handle error
  }

  // Append parameters to the message, if needed
  ret = sd_bus_message_append(m, "type", "value");
  if (ret < 0) {
    // Handle error
  }

  // Send the message and get the response
  sd_bus_message *response = NULL;
  ret = sd_bus_call(bus, m, 0, &response);
  if (ret < 0) {
    // Handle error
  }

  // Process the response, if needed
  ret = sd_bus_message_read(response, "type", &value);
  if (ret < 0) {
    // Handle error
  }

  // Clean up
  sd_bus_error_free(&error);
  sd_bus_message_unref(m);
  sd_bus_message_unref(response);
  sd_bus_unref(bus);

  return 0;
}
*/

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

    start_service_name_ = declare_parameter<std::string>("services.start", "start_service");
    stop_service_name_ = declare_parameter<std::string>("services.stop", "stop_service");
    restart_service_name_ = declare_parameter<std::string>("services.restart", "restart_service");
    query_service_name_ = declare_parameter<std::string>("services.query", "query_service");

    start_srv_ = create_service<std_srvs::srv::Empty>(start_service_name_, std::bind(&SystemctlController::start_srv, this, _1, _2));
    stop_srv_ = create_service<std_srvs::srv::Empty>(stop_service_name_, std::bind(&SystemctlController::stop_srv, this, _1, _2));
    restart_srv_ = create_service<std_srvs::srv::Empty>(restart_service_name_, std::bind(&SystemctlController::restart_srv, this, _1, _2));   
    query_srv_ = create_service<std_srvs::srv::Empty>(query_service_name_, std::bind(&SystemctlController::query_status, this, _1, _2));
  }


  void SystemctlController::start_srv(const std::shared_ptr<std_srvs::srv::Empty::Request> ,
                                    std::shared_ptr<std_srvs::srv::Empty::Response> ) {
    // open the systemd bus
    sd_bus *bus = NULL;
    sd_bus_error err = SD_BUS_ERROR_NULL;
    sd_bus_message* msg = nullptr;
    int ret = sd_bus_open_user(&bus);

    // analyze the sd_bus_default_system
    if (ret < 0) {
	    RCLCPP_INFO(get_logger(), "Could not open the User_Bus. Return: %d ", ret);     
      return;
    }

    const char *service = "MyService.service";
    const char *interface = "org.freedesktop.systemd1.Service";
    const char *method = "StartUnit";
    ret = sd_bus_call_method(bus,
                            "org.freedesktop.systemd1",             /* <service>   */
                            "/org/freedesktop/systemd1",            /* <path>      */
                            "org.freedesktop.systemd1.Manager",     /* <interface> */
                            method,                                 /* <method>    */
                            &err,                                   /* object to return error in */
                            &msg,                                   /* return message on success */
                            "ss",                                   /* <input_signature (string-string)> */
                            "myservice.service",  "replace" );      /* <arguments...> */
  
    if (ret < 0) {
	    RCLCPP_INFO(get_logger(), "Could not START the Service. Return: %d ", ret);
    }
      // Clean up
    sd_bus_error_free(&err);
    sd_bus_message_unref(msg);
    sd_bus_unref(bus);
  }


  void SystemctlController::stop_srv(const std::shared_ptr<std_srvs::srv::Empty::Request> ,
                                    std::shared_ptr<std_srvs::srv::Empty::Response> ) {
    // open the systemd bus
    sd_bus *bus = NULL;
    sd_bus_error err = SD_BUS_ERROR_NULL;
    sd_bus_message* msg = nullptr;

    int ret = sd_bus_open_user(&bus);
    if (ret < 0) {
      RCLCPP_INFO(get_logger(), "Could not open the User Bus. Return: %d ", ret);     
      return;
    }

    const char *service = "myservice.service";
    const char *interface = "org.freedesktop.systemd1.Service";
    const char *method = "StopUnit";
    ret = sd_bus_call_method(bus,
                            "org.freedesktop.systemd1",             /* <service>   */
                            "/org/freedesktop/systemd1",            /* <path>      */
                            "org.freedesktop.systemd1.Manager",     /* <interface> */
                            method,                                 /* <method>    */
                            &err,                                   /* object to return error in */
                            &msg,                                   /* return message on success */
                            "ss",                                   /* <input_signature (string-string)> */
                            "myservice.service",  "replace" );      /* <arguments...> */
    if (ret < 0) {
	    RCLCPP_INFO(get_logger(), "Could not STOP the Service. Return: %d ", ret);
    }
    // Clean up
    sd_bus_error_free(&err);
    sd_bus_message_unref(msg);
    sd_bus_unref(bus);
  }


  void SystemctlController::restart_srv(const std::shared_ptr<std_srvs::srv::Empty::Request> ,
                                        std::shared_ptr<std_srvs::srv::Empty::Response> ) {
    // open the systemd bus
    sd_bus *bus = NULL;
    sd_bus_error err = SD_BUS_ERROR_NULL;
    sd_bus_message* msg = nullptr;
   
    int ret = sd_bus_open_user(&bus);
    if (ret < 0) {
      RCLCPP_INFO(get_logger(), "Could not open the User Bus. Return: %d ", ret);     
      return;
    }
    const char *service = "myservice.service";
    const char *interface = "org.freedesktop.systemd1.Service";
    const char *method = "RestartUnit";
    ret = sd_bus_call_method(bus,
                            "org.freedesktop.systemd1",             /* <service>   */
                            "/org/freedesktop/systemd1",            /* <path>      */
                            "org.freedesktop.systemd1.Manager",     /* <interface> */
                            method,                                 /* <method>    */
                            &err,                                   /* object to return error in */
                            &msg,                                   /* return message on success */
                            "ss",                                   /* <input_signature (string-string)> */
                            "myservice.service",  "replace" );      /* <arguments...> */
    if (ret < 0) {
	    RCLCPP_INFO(get_logger(), "Could not RESTART the Service. Return: %d ", ret);
    }
    // Clean up
    sd_bus_error_free(&err);
    sd_bus_message_unref(msg);
    sd_bus_unref(bus);
  }


  void SystemctlController::query_status(const std::shared_ptr<std_srvs::srv::Empty::Request> ,
                                      std::shared_ptr<std_srvs::srv::Empty::Response> ) {
    // open the systemd bus
    sd_bus *bus = NULL;
    sd_bus_error err = SD_BUS_ERROR_NULL;
    char* state = nullptr;
    const char* property = "ActiveState";

    int ret = sd_bus_open_user(&bus);
    // analyze the sd_bus_default_system
    if (ret < 0) {
	    RCLCPP_INFO(get_logger(), "Could not open the Bus. Return: %d ", ret);     
      return;
    }

    ret = sd_bus_get_property_string(bus,
                                    "org.freedesktop.systemd1",
                                    "/org/freedesktop/systemd1/unit/myservice_2eservice",
                                    "org.freedesktop.systemd1.Unit",
                                    property,
                                    &err,
                                    &state);

    if (ret < 0)
    {
      std::string err_msg(err.message);
      sd_bus_error_free(&err);
	    RCLCPP_INFO(get_logger(), "No QUERY status. Error: %s", err_msg.c_str());     
    }
    std::string msg(state);
    RCLCPP_INFO(get_logger(), "Service Status: %s", msg.c_str());     
    // Clean up
    sd_bus_error_free(&err);
    free (state);

  }
} // namespace addons

#include <rclcpp/rclcpp.hpp>
#include <systemd/sd-bus.h>
//#include "ros2_srv/srv/Callersrv.hpp"
#include <std_srvs/srv/trigger.hpp>
#include "systemctl_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace addons{

  SystemctlController::SystemctlController(const rclcpp::NodeOptions & options) 
  : Node("systemctl_node", options) {
    const std::vector<std::string> services;
    sys_services_list = declare_parameter<std::vector<std::string>>("sys_services", services);
    start_service_name_ = declare_parameter<std::string>("services_name.start", "start_service");
    stop_service_name_ = declare_parameter<std::string>("services_name.stop", "stop_service");
    restart_service_name_ = declare_parameter<std::string>("services_name.restart", "restart_service");
    query_service_name_ = declare_parameter<std::string>("services_name.query", "query_service");

    for(std::string sys_ser : sys_services_list) {
      start_srvs_.push_back(
        create_service<std_srvs::srv::Trigger>(
          sys_ser + "/" + start_service_name_,
          [this, sys_ser](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response) { this->service_call(request, response, "StartUnit", sys_ser);}
        )
      );
      stop_srvs_.push_back(
        create_service<std_srvs::srv::Trigger>(
          sys_ser + "/" + stop_service_name_,
          [this, sys_ser](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response) { this->service_call(request, response, "StopUnit", sys_ser);}
        )
      );
      restart_srvs_.push_back(
        create_service<std_srvs::srv::Trigger>(
          sys_ser + "/" + restart_service_name_,
          [this, sys_ser](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response) { this->service_call(request, response, "RestartUnit", sys_ser);}
        )
      );
      query_srvs_.push_back(
        create_service<std_srvs::srv::Trigger>(
          sys_ser + "/" + query_service_name_,
          [this, sys_ser](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response) { this->query_status(request, response, "ActiveState", sys_ser);}
        )
      );
    }
  }

  void SystemctlController::service_call(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                          std::shared_ptr<std_srvs::srv::Trigger::Response> response,
                                          const std::string &method,
                                          std::string sys_ser) {
    sd_bus *bus = NULL;
    sd_bus_error err = SD_BUS_ERROR_NULL;
    sd_bus_message* msg = nullptr;
    int ret = sd_bus_open_user(&bus);
    
    // analyze the sd_bus_default_system
    if (ret < 0) {
	    RCLCPP_INFO(get_logger(), "Could not open the User_Bus. Return: %d ", ret);     
      return;
    }
    std::string appendix(".service");
    sys_ser.append(appendix);
    ret = sd_bus_call_method(bus,                                         /* <bus>       */
                            "org.freedesktop.systemd1",                   /* <service>   */
                            "/org/freedesktop/systemd1",                  /* <path>      */
                            "org.freedesktop.systemd1.Manager",           /* <interface> */
                            method.c_str(),                               /* <method>    */
                            &err,                                         /* object to return error in */
                            &msg,                                         /* return message on success */
                            "ss",                                         /* <input_signature (string-string)> */
                            sys_ser.c_str(),  "replace" );                /* <arguments...> */
  
    if (ret < 0) {
	    RCLCPP_INFO(get_logger(), "Could not %s the Service. Return: %d ", method.c_str(), ret);
      response->success = false;
      std::string debug_str("Service method " + method + " called unsuccessfully");
      response->message = debug_str;
    }
    // Clean up the memory:
    sd_bus_error_free(&err);
    sd_bus_message_unref(msg);
    sd_bus_unref(bus);
    response->success = true;
    std::string debug_str("Service method " + method + " called successfully");
    response->message = debug_str;
  }


  void SystemctlController::query_status(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                         std::shared_ptr<std_srvs::srv::Trigger::Response> response,
                                         const std::string &property,
                                         std::string sys_res) {
    sd_bus *bus = NULL;
    sd_bus_error err = SD_BUS_ERROR_NULL;
    char* state = nullptr;
    int ret = sd_bus_open_user(&bus);
    
    if (ret < 0) {
	    RCLCPP_INFO(get_logger(), "Could not open the Bus. Return: %d ", ret);     
      return;
    }
    std::string path_str("/org/freedesktop/systemd1/unit/" + sys_res + "_2eservice");
    ret = sd_bus_get_property_string(bus,                                                   /* <bus>        */
                                    "org.freedesktop.systemd1",                             /* <service>    */
                                    path_str.c_str(),                                       /* <path>       */
                                    "org.freedesktop.systemd1.Unit",                        /* <interface>  */
                                    property.c_str(),                                       /* <method>     */
                                    &err,                                                   /* object error */
                                    &state);                                                /* object state */
    
    if (ret < 0)
    {
      std::string err_msg(err.message);
	    RCLCPP_INFO(get_logger(), "No QUERY status. Error: %s", err_msg.c_str());
      response->success = false;
    }

    std::string msg(state);
    RCLCPP_INFO(get_logger(), "Service's Status: %s", msg.c_str());
    
    // Clean up
    sd_bus_error_free(&err);
    free (state);
    sd_bus_unref(bus);
    response->success = true;
    response->message = msg;
  }
} // namespace addons
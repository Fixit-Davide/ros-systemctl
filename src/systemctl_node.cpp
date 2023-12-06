#include <rclcpp/rclcpp.hpp>
#include <systemd/sd-bus.h>
//#include "ros2_srv/srv/Callersrv.hpp"
#include <std_srvs/srv/trigger.hpp>
#include "systemctl_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace addons
{

SystemctlController::SystemctlController(const rclcpp::NodeOptions & options)
: Node("systemctl_node", options)
{
  const std::vector<std::string> services;
  sys_services_list = declare_parameter<std::vector<std::string>>("sys_services", services);
  start_service_name_ = declare_parameter<std::string>("services_name.start", "start_service");
  stop_service_name_ = declare_parameter<std::string>("services_name.stop", "stop_service");
  restart_service_name_ =
    declare_parameter<std::string>("services_name.restart", "restart_service");
  query_service_name_ = declare_parameter<std::string>("services_name.query", "query_service");

  auto systemd_method_cb = [this](std::string systemd_unit, std::string systemd_method) {
      return [this, systemd_unit,
               systemd_method](const std::shared_ptr<std_srvs::srv::Trigger::Request> in,
               std::shared_ptr<std_srvs::srv::Trigger::Response> out) {
               this->service_call(in, out, systemd_method, systemd_unit);
             };
    };
  auto systemd_query_cb = [this](std::string systemd_unit, std::string systemd_method) {
      return [this, systemd_unit,
               systemd_method](const std::shared_ptr<std_srvs::srv::Trigger::Request> in,
               std::shared_ptr<std_srvs::srv::Trigger::Response> out) {
               this->query_status(in, out, systemd_method, systemd_unit);
             };
    };

  for (std::string sys_ser : sys_services_list) {
    start_srvs_.push_back(
      create_service<std_srvs::srv::Trigger>(
        sys_ser + "/" +
        start_service_name_, systemd_method_cb(sys_ser, "StartUnit")));
    stop_srvs_.push_back(
      create_service<std_srvs::srv::Trigger>(
        sys_ser + "/" + stop_service_name_,
        systemd_method_cb(sys_ser, "StopUnit")));
    restart_srvs_.push_back(
      create_service<std_srvs::srv::Trigger>(
        sys_ser + "/" +
        restart_service_name_, systemd_method_cb(sys_ser, "RestartUnit")));
    query_srvs_.push_back(
      create_service<std_srvs::srv::Trigger>(
        sys_ser + "/" +
        query_service_name_, systemd_query_cb(sys_ser, "ActiveState")));
  }
}

void SystemctlController::service_call(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response,
  const std::string & method,
  std::string sys_ser)
{
  sd_bus * bus = NULL;
  sd_bus_error err = SD_BUS_ERROR_NULL;
  sd_bus_message * msg = nullptr;
  int ret = sd_bus_open_user(&bus);

  // analyze the sd_bus_default_system
  if (ret < 0) {
    RCLCPP_WARN(get_logger(), "Could not open the User_Bus. Return: %d ", ret);
    return;
  }
  std::string appendix(".service");
  sys_ser.append(appendix);
  ret = sd_bus_call_method(
    bus,                                                                  /* <bus>       */
    "org.freedesktop.systemd1",                                           /* <service>   */
    "/org/freedesktop/systemd1",                                          /* <path>      */
    "org.freedesktop.systemd1.Manager",                                   /* <interface> */
    method.c_str(),                                                       /* <method>    */
    &err,                                                                 /* object to return error in */
    &msg,                                                                 /* return message on success */
    "ss",                                                                 /* <input_signature (string-string)> */
    sys_ser.c_str(), "replace");                                          /* <arguments...> */

  if (ret < 0) {
    RCLCPP_WARN(get_logger(), "%s call failed. Return: %d. ", method.c_str(), ret);
    std::string error_str(err.message);
    std::string debug_str(method + " call failed: " + error_str);
    response->success = false;
    response->message = debug_str;
    return;
  }
  RCLCPP_INFO(get_logger(), "Systemd method: %s called successfully.", method.c_str());
  // Clean up the memory:
  sd_bus_error_free(&err);
  sd_bus_message_unref(msg);
  sd_bus_unref(bus);
  std::string out_str("Systemd method: " + method + " called successfully.");
  response->success = true;
  response->message = out_str;
}

void SystemctlController::query_status(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response,
  const std::string & property,
  std::string sys_res)
{
  sd_bus * bus = NULL;
  sd_bus_error err = SD_BUS_ERROR_NULL;
  char * state = nullptr;
  int ret = sd_bus_open_user(&bus);

  if (ret < 0) {
    RCLCPP_WARN(get_logger(), "Could not open the Bus. Return: %d ", ret);
    return;
  }
  std::string path_str("/org/freedesktop/systemd1/unit/" + sys_res + "_2eservice");
  ret = sd_bus_get_property_string(
    bus,                                                                                    /* <bus>        */
    "org.freedesktop.systemd1",                                                             /* <service>    */
    path_str.c_str(),                                                                       /* <path>       */
    "org.freedesktop.systemd1.Unit",                                                        /* <interface>  */
    property.c_str(),                                                                       /* <method>     */
    &err,                                                                                   /* object error */
    &state);                                                                                /* object state */

  if (ret < 0) {
    std::string error_str(err.message);
    RCLCPP_WARN(
      get_logger(), "Could NOT query the status of the service. Error: %s. Return: %d.",
      error_str.c_str(), ret);
    response->success = false;
    return;
  }

  std::string state_str(state);
  std::string out_str("Systemd service: " + sys_res + " status: " + state_str);
  RCLCPP_INFO(get_logger(), "Service %s Status: %s.", sys_res.c_str(), state_str.c_str());
  // Clean up
  sd_bus_error_free(&err);
  sd_bus_unref(bus);
  free(state);
  response->success = true;
  response->message = out_str;
}
} // namespace addons

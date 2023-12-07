# ROS2-systemctl Node: 
**systemctl** is a command line utility that is used to control and manage system services on Linux operating system.
This Node enables the communication with **systemctl** using basic ROS2 services (std_srvs::srv::Request). It allows the user to start, stop, restart and visualize the status of a specific system service.
## How to Use:
- In order to function correctly as a normal user, the custom Systemd unit file has to be generated in the `~/.config/systemd/user/` directory.
- In the params file the names of the systemd services can be set as a simple list **omitting** the .service extension of the name.
- Similarly, it is possible to change the name of the ROS2 services in order to make node integration and customization easier.
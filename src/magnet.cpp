#include <ros/package.h>
#include <ros/ros.h>
#include <ur_msgs/SetIO.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "magnet_control");
    ros::NodeHandle node_handle;
    ros::ServiceClient ur_io_service;

    int input_command;
    node_handle.getParam("/magnet_node/command", input_command);

    std::cout << input_command << std::endl;

    std::string robot_namespace;
    node_handle.getParam("/magnet_node/robot_namespace", robot_namespace);
    std::string ref_topic = robot_namespace + "/ur_driver/set_io";

    std::cout << ref_topic << std::endl;

    ur_io_service = node_handle.serviceClient<ur_msgs::SetIO>("/ur5/ur_driver/set_io");

    std::string operation = "";

    ur_msgs::SetIO set_io;
    set_io.request.fun = set_io.request.FUN_SET_DIGITAL_OUT;
    set_io.request.pin = 0;

    if (input_command == 1) {
        operation = "magnet_on";
    } else if (input_command == 0) {
        operation = "magnet_off";
    } else {
        ROS_ERROR_STREAM("Failed to call Robot Set I/O service for input command. Available commands: 1 for activation; 0 for deactivation");
        return 1;
    }

    set_io.request.state = input_command;

    if (ur_io_service.call(set_io))
        ROS_INFO_STREAM("Operation " + operation + " succeded");
    else
        ROS_ERROR_STREAM("Failed to call Robot Set I/O service for operation " + operation);

    ros::waitForShutdown();
    return 0;
}

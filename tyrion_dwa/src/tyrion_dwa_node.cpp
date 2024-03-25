#include <tyrion_dwa.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_executor");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ROS_INFO("Creating DWA_controller");
    Dwa3d controller(nh, nh_private);
    ROS_INFO("DWA_controller created");
    controller.idle();
    return 0;
}
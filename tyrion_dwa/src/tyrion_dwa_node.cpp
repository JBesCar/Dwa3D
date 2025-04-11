#include <tyrion_dwa.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "tyrion_dwa_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("tyrion_dwa");
    // Load constant parameters
    double R_drone; //[m], Default = 0.4
    double T; //Control period [s]
    double delta_t;
    double vx_step; //Search Space Discretization vx[m/s], Default = 0.05
    double vz_step; //Search Space Discretization vz[m/s], Default = 0.05
    double w_step; // Search Space Discrezitation w[rad/s], Default = Pi/36 = 5º/s
    double aLin; // Maximum Linear Acceleration [m/ss], Default = 1.0
    double aAng; // Maximum Angular Acceleration [rad/ss], Default = Pi/1.8 = 100º/s
    nh.param("R_drone", R_drone, 0.4);
    nh.param("T_control", T, 0.1);
    nh.param("delta_t", delta_t, 1.0);
    nh.param("vx_step", vx_step, 0.05);
    nh.param("vz_step", vz_step, 0.05);
    nh.param("w_step", w_step, PI/72);
    nh.param("aLin", aLin, 1.0);
    nh.param("aAng", aAng, PI/1.8);
    // Create the DWA controller
    ROS_INFO("Creating DWA_controller");
    Dwa3d controller(nh, nh_private, R_drone, T, delta_t,
                    vx_step, vz_step, w_step, 
                    aLin, aAng);
    ROS_INFO("DWA_controller created");
    // Execute the controller  
    controller.idle();
    return 0;
}
#include <tyrion_dwa.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_executor");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("tyrion_dwa");
    // Load constant parameters
    double R_drone; //[m], Default = 0.4
    double T; //Periodo de control [s]
    double delta_t;
    double vx_step, vz_step; //Resolución de discretización del espacio de búsqueda en xz[m/s], Default = 0.05
    double w_step; // Resolucion de discretizacion del espacio de busqueda en yaw (5º), Default = Pi/36
    double aLin; // Aceleracion lineal máxima [m/ss], Default = 1.0
    double aAng; // Aceleracion angular maxima [rad/ss] 10º, Default = Pi/1.8
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
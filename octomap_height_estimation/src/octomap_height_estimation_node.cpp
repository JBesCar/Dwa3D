#include <octomap_height_estimation.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "floor_height_estimator");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    HeightEstimator estimator(nh, nh_private);
    // Execute the controller  
    estimator.loop();
    return 0;
}
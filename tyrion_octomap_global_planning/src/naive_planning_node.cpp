#include <naive_planning.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "naive_plan_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    //Take off and go to the goal
    NaivePlanner drone(std::ref(node_handle));
    //drone.takeoff();
    drone.run();
    
    return 0;
}

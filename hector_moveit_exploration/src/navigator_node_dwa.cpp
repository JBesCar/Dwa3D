#include <Navigator_dwa.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "navigator_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    //Take off and go to the goal
    Navigator_dwa drone(std::ref(node_handle));
    //drone.takeoff();
    drone.run();
    
    return 0;
}

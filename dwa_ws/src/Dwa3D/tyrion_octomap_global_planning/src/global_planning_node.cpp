#include <global_planning.h>

int main(int argc, char ** argv)
{   
    ros::init(argc, argv, "global_planner");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    //Plan
    GlobalPlanner planner(std::ref(node_handle));
    planner.run();


    //planWithSimpleSetup();

    return 0;
}


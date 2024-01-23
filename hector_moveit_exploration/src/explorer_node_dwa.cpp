#include <Explorer_dwa.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "hector_explorer");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");
    geometry_msgs::Pose _goal;
    if (argc > 0){
        std::string s = argv[1];
        if(s.compare("tunel") == 0)         {_goal.position.x = 113, _goal.position.y = 25, _goal.position.z = 1.5;} 
        else if(s.compare("cueva") == 0)    {_goal.position.x = 0.0, _goal.position.y = 5.0, _goal.position.z = 1.0;} 
        else if(s.compare("cueva2") == 0)   {_goal.position.x = 0.0, _goal.position.y = 15, _goal.position.z = 6.0;} 
        else if(s.compare("prueba") == 0)   {_goal.position.x = 55, _goal.position.y = 0, _goal.position.z = 1.5;}
        else if(s.compare("lab") == 0)      {_goal.position.x = 9.5, _goal.position.y = 0.0, _goal.position.z = 1.0;}
        else if(s.compare("arena")== 0)     {_goal.position.x = 3.0, _goal.position.y = 0.0, _goal.position.z = 0.0;}
    } else {_goal.position.x = 113, _goal.position.y = 25, _goal.position.z = 1.5;}
    Quadrotor_dwa quad(std::ref(node_handle));
    std::string d = argv[2];
    quad.takeoff();
    std::cout<<"Goal: "<<_goal<<std::endl;
    quad.run(_goal);
    return 0;
}
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/SetMavFrame.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

mavros_msgs::State current_state;
mavros_msgs::ExtendedState current_extended_state;
std::string current_order;
geometry_msgs::PoseStamped current_pose;
float rel_z_takeoff = 1.0;

ros::Subscriber state_sub, order_sub, pose_sub;
ros::Publisher cmd_vel_pub;
ros::ServiceClient arming_client, set_cmd_vel_frame, set_mode_client; 

void stateCallback(const mavros_msgs::State::ConstPtr& msg);
void extendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr &msg);
void orderCallback(const std_msgs::String::ConstPtr& msg);
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
bool tryOffboard(void);
bool takeOff(float z_setpoint);
bool land(void);
bool disarm(void);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tyrion_commander");
    ros::NodeHandle nh;

    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, stateCallback);

    order_sub = nh.subscribe<std_msgs::String>("order", 10, orderCallback);

    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, poseCallback);

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    set_cmd_vel_frame = nh.serviceClient<mavros_msgs::SetMavFrame>("mavros/setpoint_velocity/mav_frame");

    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //TO-DO Parametriz delta_z_takeoff

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    // while(ros::ok() && !current_state.connected){
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    bool offboard = false;
    bool height_reached = false;
    float z_setpoint;

    while(ros::ok()){
        //Process order
        if(current_order.compare(std::string("TAKEOFF")) == 0){
            ros::Rate try_offboard_rate(1.0);
            //Go offboard before takeOff
            if(!offboard){
                ROS_WARN("TRY OFFBOARD");
                try_offboard_rate.sleep();
                offboard = tryOffboard();
                z_setpoint = current_pose.pose.position.z + rel_z_takeoff;
            }else{
                //TakeOff with velocity cmds in Offboard 
                height_reached = takeOff(z_setpoint);
            }
            if(height_reached){
                current_order = "IDLE";
                offboard = false;
                height_reached = false;
                geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                cmd_vel.linear.z = 0;
                cmd_vel.angular.x = 0;
                cmd_vel.angular.y = 0;
                cmd_vel.angular.z = 0; //5.0/180 * M_PI;
                cmd_vel_pub.publish(cmd_vel);
            }
        }else if(current_order == "LAND"){
            if(land()){
                disarm();
                current_order = "IDLE";
            }
        }else if(current_order == "IDLE"){
            //ROS_WARN("IDLE");
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.linear.z = 0;
            cmd_vel.angular.x = 0;
            cmd_vel.angular.y = 0;
            cmd_vel.angular.z = 0; //5.0/180 * M_PI;
            cmd_vel_pub.publish(cmd_vel);
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}





void stateCallback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void extendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr &msg)
{
    current_extended_state = *msg;
}
void orderCallback(const std_msgs::String::ConstPtr& msg){
    current_order = std::string(msg->data);
    ROS_WARN("ORDER RECIEVED");
}
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

bool tryOffboard(void){
    bool success = false;
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    //Create the twist message
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0; //5.0/180 * M_PI;
    
    //Stablish the cmd_vel frame
    mavros_msgs::SetMavFrame set_frame_msg;
    set_frame_msg.request.mav_frame = set_frame_msg.request.FRAME_BODY_NED;
    set_cmd_vel_frame.call(set_frame_msg);

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        cmd_vel_pub.publish(cmd_vel);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if( current_state.mode != "OFFBOARD"){
        if( set_mode_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
        }else{
            ROS_WARN("OFFBOARD DENIED, TRYING AGAIN");
        }
    } else {
        if( !current_state.armed){
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
                success = true;
            }else{
                ROS_WARN("NOT ABLE TO ARM, TYRING AGAIN");
            }
        }
    }
    cmd_vel_pub.publish(cmd_vel);
    return success;
}

bool takeOff(float z_setpoint){
    bool success = false;
    //Create the twist message
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0.25;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0; //5.0/180 * M_PI;
    cmd_vel_pub.publish(cmd_vel);

    if(fabs(z_setpoint - current_pose.pose.position.z) < 0.1){
        success = true;
    }

    return success;
}

bool land(void)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = -0.2;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;
    cmd_vel_pub.publish(cmd_vel);

    return current_extended_state.landed_state == current_extended_state.LANDED_STATE_ON_GROUND;
}

bool disarm(void)
{
    if (current_state.mode == "OFFBOARD")
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.linear.z = 0;
        cmd_vel.angular.x = 0;
        cmd_vel.angular.y = 0;
        cmd_vel.angular.z = 0;
        cmd_vel_pub.publish(cmd_vel);
    }
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = false;
    arming_client.call(arm_cmd);
    return arm_cmd.response.success;
}
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
#include <sensor_msgs/Range.h>



class TyrionCommander{

    public:
        mavros_msgs::State current_state;
        mavros_msgs::ExtendedState current_extended_state;
        std::string current_order, last_order;
        geometry_msgs::PoseStamped current_pose, goal;
        float z_landing = 0.3;
        float range;
        float qz0, qw0;

        ros::Subscriber state_sub, order_sub, pose_sub, goal_sub, range_sub;
        ros::Publisher cmd_vel_pub, landing_target_pub, pose_pub, order_pub;
        ros::ServiceClient arming_client, set_cmd_vel_frame, set_mode_client; 

        
        void stateCallback(const mavros_msgs::State::ConstPtr& msg);
        void extendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr &msg);
        void orderCallback(const std_msgs::String::ConstPtr& msg);
        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        bool tryOffboard(void);
        bool takeOff(float z_setpoint);
        bool land(void);
        bool disarm(void);
        void precision_landing(void);
        void pidApproach(const geometry_msgs::PoseStamped::ConstPtr& goal, 
                        const geometry_msgs::PoseStamped::ConstPtr& current_pose);
        void rangeCallback(const sensor_msgs::Range::ConstPtr& msg);
        void loop(void);
        

        TyrionCommander(ros::NodeHandle nh){
            state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &TyrionCommander::stateCallback, this);

            order_sub = nh.subscribe<std_msgs::String>("order", 10, &TyrionCommander::orderCallback, this);

            pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &TyrionCommander::poseCallback, this);

            range_sub = nh.subscribe<sensor_msgs::Range>("mavros/distance_sensor/mini_tf_pub", 1, &TyrionCommander::rangeCallback, this);

            order_pub = nh.advertise<std_msgs::String>("order", 1);

            goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("goal", 10, &TyrionCommander::goalCallback, this);

            cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);

            landing_target_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/landing_target/pose", 1);

            pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);

            arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

            set_cmd_vel_frame = nh.serviceClient<mavros_msgs::SetMavFrame>("mavros/setpoint_velocity/mav_frame");

            set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

            nh.param("Ki_xy_land", Ki_xy_land, 0.2);
            nh.param("Kp_xy_land", Kp_xy_land, 0.5);
            nh.param("Kp_yaw_land", Kp_yaw_land, 1.0);
            nh.param("Ki_yaw_land", Ki_yaw_land, 0.1);
            nh.param("Ki_z_land", Ki_z_land, 0.2);
            nh.param("Kp_z_land", Kp_z_land, 1.0);
            nh.param("VMAX_XY_LAND", VMAX_XY_LAND, 0.15);
            nh.param("VMIN_XY_LAND", VMIN_XY_LAND, -0.15);
            nh.param("VMAX_Z_LAND", VMAX_Z_LAND, 0.0);
            nh.param("VMIN_Z_LAND", VMIN_Z_LAND, -0.1);
            nh.param("W_MAX_LAND", W_MAX_LAND, 0.5);
            nh.param("W_MIN_LAND", W_MIN_LAND, -0.5);
            nh.param("rel_z_takeoff", rel_z_takeoff, 0.7);

        }

        private:
            double Ki_xy_land;
            double Kp_xy_land;
            double Kp_yaw_land;
            double Ki_yaw_land;
            double Ki_z_land;
            double Kp_z_land;
            double VMAX_XY_LAND;
            double VMIN_XY_LAND;
            double VMAX_Z_LAND;
            double VMIN_Z_LAND;
            double W_MAX_LAND;
            double W_MIN_LAND;
            double rel_z_takeoff;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tyrion_commander");
    ros::NodeHandle nh;
    TyrionCommander commander(nh);
    commander.loop();

    return 0;
}

    
void TyrionCommander::loop(){
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
                z_landing = range;//current_pose.pose.position.z;
                qz0 = current_pose.pose.orientation.z; 
                qw0 = current_pose.pose.orientation.w;
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
        }else if(current_order == "NAVIGATE"){
            if(current_state.mode != "OFFBOARD"){
                bool offboard = tryOffboard();
            }
        /*}else if(current_order == "LAND"){
            //precision_landing();
            if(current_state.mode != "OFFBOARD"){
                bool offboard = tryOffboard();
            }else{
                if(land()){
                    disarm();
                    current_order = "IDLE";
                }
            }*/
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

        }else if(current_order == "LAND"){
            if(current_state.mode != "OFFBOARD"){
                bool offboard = tryOffboard();
            }else{
                float dx = goal.pose.position.x - current_pose.pose.position.x; //+ en vez de menos porque current_pose está en frame "odom_ned"
                float dy = goal.pose.position.y - current_pose.pose.position.y; //+ en vez de menos porque current_pose está en frame "odom_ned"
                float distance = sqrt((dx*dx + dy*dy));
                //if(distance > 0.1){
                    //Stablish the cmd_vel frame
                    mavros_msgs::SetMavFrame set_frame_msg;
                    set_frame_msg.request.mav_frame = set_frame_msg.request.FRAME_LOCAL_NED;
                    set_cmd_vel_frame.call(set_frame_msg);
                    pidApproach(boost::make_shared<const geometry_msgs::PoseStamped>(goal), 
                                boost::make_shared<const geometry_msgs::PoseStamped>(current_pose));
                //}else{
                //    if(land()){
                //        disarm();
                //        current_order = "IDLE";
                //    }
                //}
            }  
        }else if(current_order == "HOVER"){
            mavros_msgs::SetMode position_set_mode;
            position_set_mode.request.custom_mode = "POSCTL";

            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = true;

            if(current_state.mode != "POSCTL"){
                if(set_mode_client.call(position_set_mode) &&
                    position_set_mode.response.mode_sent){
                    ROS_INFO("Position enabled");
                }
            }
        }

        if(current_order.compare(last_order) != 0){
            last_order = std::string(current_order);
            std_msgs::String msg;
            msg.data = current_order;
            order_pub.publish(msg);
        }
        ros::spinOnce();
        rate.sleep();
    }
}

void TyrionCommander::rangeCallback(const sensor_msgs::Range::ConstPtr& msg){
    range = msg->range;
}


void TyrionCommander::pidApproach(const geometry_msgs::PoseStamped::ConstPtr& goal, 
    const geometry_msgs::PoseStamped::ConstPtr& current_pose) {

    const float DELTA_T = 0.05;

    static float integral_sum_x = 0;
    static float integral_sum_y = 0;
    static float integral_sum_z = 0;
    static float integral_sum_yaw = 0;

    double roll, pitch, yaw_goal, yaw_current;
    tf::Quaternion q_current(0, 0, current_pose->pose.orientation.z, current_pose->pose.orientation.w);
    tf::Matrix3x3 m_current(q_current);
    m_current.getRPY(roll, pitch, yaw_current);

    tf::Quaternion q_goal(0, 0, qz0, qw0);
    tf::Matrix3x3 m_goal(q_goal);
    m_goal.getRPY(roll, pitch, yaw_goal);


    float e_kx, e_ky, e_kz, e_kyaw;
    e_kx = goal->pose.position.x - current_pose->pose.position.x; // Se suma en vez de restar 
    e_ky = goal->pose.position.y - current_pose->pose.position.y; // porque current_pose está en odom_ned
    e_kz = z_landing - current_pose->pose.position.z;//- range;

    e_kyaw = yaw_goal - yaw_current;
    //e_kyaw += + M_PI/2;
    while(e_kyaw > M_PI){
        e_kyaw -= 2 * M_PI;
    }
    while(e_kyaw <= -M_PI){
        e_kyaw += 2 * M_PI;
    }
    

    std::cout << "e_kx: " << e_kx << std::endl;
    std::cout << "e_ky: " << e_ky << std::endl;
    std::cout << "e_kz: " << e_kz << std::endl;
    std::cout << "e_yaw: " << e_kyaw << std::endl;

    //PID X
    integral_sum_x += e_kx * DELTA_T;
    float v_raw_x = Kp_xy_land * e_kx + Ki_xy_land * integral_sum_x;
    float vx = v_raw_x;
    if(v_raw_x > VMAX_XY_LAND){
        vx = VMAX_XY_LAND;
        integral_sum_x -= e_kx * DELTA_T;
    }else if(v_raw_x < VMIN_XY_LAND){
        vx = VMIN_XY_LAND;
        integral_sum_x -= e_kx * DELTA_T;
    }

    //PID Y
    integral_sum_y += e_ky * DELTA_T;
    float v_raw_y = Kp_xy_land * e_ky + Ki_xy_land * integral_sum_y;
    float vy = v_raw_y;
    if(v_raw_y > VMAX_XY_LAND){
        vy = VMAX_XY_LAND;
        integral_sum_y -= e_ky * DELTA_T;
    }else if(v_raw_y < VMIN_XY_LAND){
        vy = VMIN_XY_LAND;
        integral_sum_y -= e_ky * DELTA_T;
    }

    //PID Z
    integral_sum_z += e_kz * DELTA_T;
    float v_raw_z = Kp_z_land * e_kz + Ki_z_land * integral_sum_z;
    float vz = v_raw_z;
    if(v_raw_z > VMAX_Z_LAND){
        vz = VMAX_Z_LAND;
        integral_sum_z -= e_kz * DELTA_T;
    }else if(v_raw_z < VMIN_Z_LAND){
        vz = VMIN_Z_LAND;
        integral_sum_z -= e_kz * DELTA_T;
    }

    //PID YAW
    integral_sum_yaw += e_kyaw * DELTA_T;
    float w_raw = Kp_yaw_land * e_kyaw + Ki_yaw_land * integral_sum_yaw;
    float w = w_raw;
    if(w_raw > W_MAX_LAND){
        w = W_MAX_LAND;
        integral_sum_yaw -= e_kyaw * DELTA_T;
    }else if(w < W_MIN_LAND){
        w = W_MIN_LAND;
        integral_sum_yaw -= e_kyaw * DELTA_T;
    }

    //Publish msg
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = vx;
    cmd_vel.linear.y = vy;
    cmd_vel.linear.z = vz;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = w;
    cmd_vel_pub.publish(cmd_vel);
    

    std::cout << "Vx: " << vx << std::endl;
    std::cout << "Vy: " << vy << std::endl;
    std::cout << "Vz: " << vz << std::endl;
    //std::cout << "w: " << w << std::endl;
}

void TyrionCommander::stateCallback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void TyrionCommander::extendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr &msg)
{
    current_extended_state = *msg;
}
void TyrionCommander::orderCallback(const std_msgs::String::ConstPtr& msg){
    current_order = std::string(msg->data);
    ROS_WARN("ORDER RECIEVED");
}
void TyrionCommander::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

void TyrionCommander::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    goal = *msg;
}

bool TyrionCommander::tryOffboard(void){
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

bool TyrionCommander::takeOff(float z_setpoint){
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

bool TyrionCommander::land(void)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = -0.1;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;
    cmd_vel_pub.publish(cmd_vel);

    return current_extended_state.landed_state == current_extended_state.LANDED_STATE_ON_GROUND;
}

void TyrionCommander::precision_landing(void){
    geometry_msgs::PoseStamped landing_target;
    /*if(goalRecieved){
        landing_target.pose.position.x = goal.pose.position.x;
        landing_target.pose.position.y = goal.pose.position.y;
        landing_target.pose.position.z = goal.pose.position.z - 1;
        landing_target.header.frame_id = "odom";
    }else{*/
    landing_target.pose.position.x = -goal.pose.position.x;//current_pose.pose.position.x;
    landing_target.pose.position.y = -goal.pose.position.y;//current_pose.pose.position.y;
    landing_target.pose.position.z = 0.3;
    landing_target.header.frame_id = "odom_ned";
    //}
    landing_target.header.stamp = ros::Time::now();
    landing_target_pub.publish(landing_target);
    mavros_msgs::SetMode landing_set_mode;

    if(current_state.mode != "AUTO.PRECLAND"){
        landing_set_mode.request.custom_mode = "AUTO.PRECLAND";
        bool service_called = set_mode_client.call(landing_set_mode);
        ROS_WARN("TRYING TO ENABLE PRECISION LANDING");
    }

}

bool TyrionCommander::disarm(void)
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
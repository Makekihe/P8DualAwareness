#include <ros/ros.h>
#include <string>
#include <iostream>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//Function declarations
void send_goal_position();      // Sends a goal position to the move_base
void send_to_arduino();         // Sends an integer to the arduino controlling the lights
double distance_to_human();     // Calculates the distance between the robot and human
void change_vel();              // Changes the velocity of the robot based on distance_to_human

class robot_behaviour{

    private:
        ros::NodeHandle nh;

        ros::Publisher pub_ard;  //Publisher to the arduino (controlling signalling)

        ros::Subscriber sub_humanpose;  //Subscriber to the position of the human
        ros::Subscriber sub_teb_direction;  //Subscriber to the TEB local planner (used for checking when we turnin')
        ros::Subscriber sub_amclPose;

        //Variables
        double robot_pose_x;
        double robot_pose_y;
        double human_pose_x;
        double human_pose_y;

    public:

        robot_behaviour(){
            ROS_INFO("Object is being created");
            pub_ard = nh.advertise<std_msgs::Int8>("arduino", 10);

            sub_humanpose = nh.subscribe("coordinates", 10, &robot_behaviour::humanpose_callback, this);
            sub_teb_direction = nh.subscribe("direction", 10, &robot_behaviour::teb_direction_callback, this);
            sub_amclPose = nh.subscribe("amcl_pose", 10, &robot_behaviour::robot_pose_callback, this);            
        }

        //Store the humans most recent pose
        void humanpose_callback(geometry_msgs::PoseArray msgs){
            human_pose_x = msgs.poses[0].position.x;
            human_pose_y = msgs.poses[0].position.y;
            change_vel();
        }

        //Store the robots most recent pose
        void robot_pose_callback(geometry_msgs::PoseStamped msgs){
            robot_pose_x = msgs.pose.position.x;
            robot_pose_y = msgs.pose.position.y;
            change_vel();
        }

        // Send an integer to the arduino to signal according to local planner information
        void teb_direction_callback(std_msgs::String msgs){
            std::string msgs_teb = msgs.data; 

            if (msgs_teb == "right"){
                send_to_arduino(1, pub_ard);                    
            }

            if (msgs_teb == "left"){
                send_to_arduino(2, pub_ard);                    
            }

            if (msgs_teb == "going straight"){
                send_to_arduino(5, pub_ard);                    
            }
        }

        //Send a predefined position to the robot
        void send_goal_position(){
            MoveBaseClient ac("move_base",true);
            move_base_msgs::MoveBaseGoal goalPosition;

            goalPosition.target_pose.header.frame_id = "map";
            goalPosition.target_pose.header.stamp = ros::Time::now();
            goalPosition.target_pose.pose.position.x = 5.0;
            goalPosition.target_pose.pose.position.y = 5.0;

            tf::Quaternion quat;
            quat.setRPY(0.0, 0.0, 1.57);
            tf::quaternionTFToMsg(quat, goalPosition.target_pose.pose.orientation);
            ac.sendGoal(goalPosition);
        }

        //Publishing an integer to the arduino
        void send_to_arduino(int x, ros::Publisher pub_ard){
        std_msgs::Int8 arduinoMessage;
        arduinoMessage.data = x;
        pub_ard.publish(arduinoMessage);
        }

        //Calculate the distance between the current pose of the human and the robot
        double distance_to_human(double x_h, double y_h, double x_r, double y_r){
            double length = sqrt(pow(human_pose_x-robot_pose_x, 2)+pow(human_pose_y+robot_pose_y, 2));
            return length;   
        }

        //Change the robots velocity depending on the distance to the human
        void change_vel(){    
            double dist_to_human = distance_to_human(human_pose_x, human_pose_y, robot_pose_x, robot_pose_y);

            if (dist_to_human < 3.5){
                if (dist_to_human < 1.08){
                send_to_arduino(4, pub_ard);
                system("rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS max_vel_x 0.0");
            }
                if(dist_to_human > 1.08){
                send_to_arduino(7, pub_ard);
                system("rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS max_vel_x 0.3");
            }

            else{
                system("rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS max_vel_x 0.8");
            }
            }
        }

};




int main(int argc, char* argv[])
{
    ros::init(argc, argv, "robot_behaviour_node");
    robot_behaviour robot_behaviour_object;

    ROS_INFO("ROSNODE initialised");

    while(ros::ok()){
        ros::spinOnce();
    }

    robot_behaviour_object.send_to_arduino(8, robot_behaviour::pub_ard);
    return 0;
}
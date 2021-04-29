//TODO Use position info to determine if the robot is rotating in place or moving
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <numeric>

#include "ros/ros.h"
#include "tf/tf.h"
#include "geometry_msgs/PoseStamped.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"

#define PI 3.14159265

class determine_direction{
    private:
        ros::Publisher pub;
        ros::Subscriber sub;
        ros::Subscriber sub_status;
        ros::NodeHandle nh;
        std::vector<float> yaw_moving_average;
    public:
        determine_direction(){
            std::cout << "Object is being created" << std::endl;
            pub=nh.advertise<std_msgs::String>("direction", 10);;
            sub=nh.subscribe("/move_base/DWAPlannerROS/local_plan", 10, &determine_direction::callback, this);
            sub_status=nh.subscribe("/move_base/status", 10, &determine_direction::callback_status, this);
        }

        void callback_status(const actionlib_msgs::GoalStatusArrayConstPtr& msg){
            if(!msg->status_list.empty()){
                uint goal_status=msg->status_list.back().status;

                if(goal_status==3){
                    yaw_moving_average.clear();
                }
            }
        }

        void callback(const nav_msgs::PathConstPtr& msg){
            tf::Quaternion q_current(
                msg->poses[0].pose.orientation.x,
                msg->poses[0].pose.orientation.y,
                msg->poses[0].pose.orientation.z,
                msg->poses[0].pose.orientation.w);
            
            tf::Quaternion q_goal(
                msg->poses.back().pose.orientation.x,
                msg->poses.back().pose.orientation.y,
                msg->poses.back().pose.orientation.z,
                msg->poses.back().pose.orientation.w);
            
            //available if ever needed
            /*float x_current=msg->poses[0].pose.position.x;
            float y_current=msg->poses[0].pose.position.y;
            float x_goal=msg->poses.back().pose.position.x;
            float y_goal=msg->poses.back().pose.position.y;
            float x_delta=x_goal-x_current;
            float y_delta=y_goal-y_current;
            float delta_length=sqrt(pow(x_delta,2.0f)+pow(y_delta,2.0f));
            std::cout<<"delta_length "<<delta_length<<" x_delta "<<x_delta<<" y_delta "<<y_delta<<std::endl;
            bool isTurningOnTheSpot=false;
            if(delta_length<10.0f) isTurningOnTheSpot=true;
            int msg_length=msg->poses.size();*/

            double q_angle=q_current.angleShortestPath(q_goal);
            double q_angle_deg=q_angle*180.0/PI;
            yaw_moving_average.push_back(q_angle_deg);

            tf::Matrix3x3 m_goal(q_goal);
            tf::Matrix3x3 m_current(q_current);
            double roll, pitch, yaw;
            m_current.getRPY(roll,pitch,yaw);
            double yaw_current=yaw*180.0/PI;
            m_goal.getRPY(roll,pitch,yaw);
            double yaw_goal=yaw*180.0/PI;
            double yaw_delta=yaw_goal-yaw_current;
            //std::cout<<"yaw_delta "<<yaw_delta<<std::endl;

            std_msgs::String direction_msg;
            if(yaw_moving_average.size()==4){   
                yaw_moving_average.erase(yaw_moving_average.begin());
                float yaw_average=std::accumulate(yaw_moving_average.begin(),yaw_moving_average.end(),0.0f)/yaw_moving_average.size();
                //std::cout<<"yaw moving average "<<yaw_average<<std::endl;

                if(std::fabs(yaw_average)>30.0f){
                    if(yaw_delta>0.0) {
                        //std::cout<<"Left and Yaw_average is "<<yaw_average<<std::endl;
                        //std::cout<<"Left"<<std::endl;
                        direction_msg.data="left";
                    } else {
                        //std::cout<<"Right and Yaw_average is "<<yaw_average<<std::endl;
                        //std::cout<<"Right"<<std::endl;
                        direction_msg.data="right";
                    }
                } else {
                    //std::cout<<"Going straight and Yaw_average is "<<yaw_average<<std::endl;
                    //std::cout<<"Straight"<<std::endl;
                    direction_msg.data="going straight";
                }

                pub.publish(direction_msg);
            }
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "determine_direction_class");
    determine_direction determine_direction_object;
    
    while(ros::ok()){
        ros::spinOnce();
        //sleep(1);
    }

return 0;
}

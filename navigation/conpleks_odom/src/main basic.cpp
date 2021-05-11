#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Imu.h"
#include "actionlib_msgs/GoalStatusArray.h"

class conpleks_odom{
    private:
        ros::NodeHandle n;
        nav_msgs::Odometry odom;
        geometry_msgs::TwistStamped hmi_vel;
        ros::Publisher odom_pub, hmi_vel_pub;
        ros::Subscriber vel_sub;
        tf::TransformBroadcaster odom_broadcaster;
        geometry_msgs::TransformStamped odom_trans;
        long double x=0.0L, y=0.0L, th=0.0L; //initial pose
        long double v=0.0L, omega=0.0L; //velocity 
        
    public:
        ros::Time previous_time, current_time;
        
        conpleks_odom(){
            std::cout << "Object is being created" << std::endl;
            odom_pub=n.advertise<nav_msgs::Odometry>("conpleks_odom", 50);
            hmi_vel_pub=n.advertise<geometry_msgs::TwistStamped>("hmi/cmd_vel",50);    
            vel_sub=n.subscribe("cmd_vel", 10, &conpleks_odom::vel_callback, this); 
        
            for (size_t i = 0; i < 36; i++) {
                odom.pose.covariance[i]=0.0;
                odom.twist.covariance[i]=0.0;
            }

            odom.pose.covariance[0]=0.1; //0.1
            odom.pose.covariance[7]=0.1; //0.1
            odom.pose.covariance[35]=0.4; //0.2
            odom.pose.covariance[14]=999999999999.0;
            odom.pose.covariance[21]=999999999999.0;
            odom.pose.covariance[28]=999999999999.0;

            odom.twist.covariance[0]=0.1;
            odom.twist.covariance[7]=0.1;
            odom.twist.covariance[35]=0.1;
            odom.twist.covariance[14]=999999999999.0;
            odom.twist.covariance[21]=999999999999.0;
            odom.twist.covariance[28]=999999999999.0;

            std::cout << "Waiting for a message" << std::endl;
        }

        void vel_callback(const geometry_msgs::Twist::ConstPtr& msg){
            //std::cout<<"Velocity Callback"<<std::endl;
            v=msg->linear.x;
            omega=msg->angular.z;

            hmi_vel.header.stamp=current_time;
            hmi_vel.twist.linear.x=v;
            hmi_vel.twist.angular.z=omega;

            hmi_vel_pub.publish(hmi_vel);
        }

        void calc_odom(){
            double dt=(current_time-previous_time).toSec();
            long double delta_x=v*cos(th)*dt;
            long double delta_y=v*sin(th)*dt;
            long double delta_th=omega*dt;

            x += delta_x;
            y += delta_y;
            th += delta_th;

            if(th>M_PI) {
                long double looped_th=th-M_PI;
                th=-M_PI+looped_th;
            }

            if(th<-M_PI) {
                long double looped_th=th+M_PI;
                th=M_PI+looped_th;
            }
        }

        void pub_odom(){
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_footprint";

            odom_trans.transform.translation.x = x;
            odom_trans.transform.translation.y = y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;

            odom_broadcaster.sendTransform(odom_trans);

            odom.header.stamp = current_time;
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_footprint";

            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.position.z = 0.0;

            odom.pose.pose.orientation = odom_quat;

            odom.twist.twist.linear.x = v;
            odom.twist.twist.angular.z = omega;

            odom_pub.publish(odom);
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "conpleks_odom");
    conpleks_odom odom_object;

    odom_object.current_time=ros::Time::now();
    odom_object.previous_time=ros::Time::now();

    ros::Rate r(10000);
    while(ros::ok()){
        odom_object.current_time=ros::Time::now();
        ros::spinOnce();
        odom_object.calc_odom();
        odom_object.pub_odom();
        odom_object.previous_time=odom_object.current_time;
        r.sleep();
    }

return 0;
}
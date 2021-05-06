#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"


class calculateDistance{
    private:
        ros::Subscriber sub;
        ros::NodeHandle nh;
        float shortestDistance = 100;
        float shortestReading = 100;
	std::ofstream myfile;


    public:
        calculateDistance(){
            ROS_INFO("Object is being created");   
            //Change the topic of the subscriber to fit the RPLidar
            sub = nh.subscribe("scan", 10, &calculateDistance::callback, this);
        }


        void callback(const sensor_msgs::LaserScan::ConstPtr& msgs){
            //Determine the smalles non-zero number in the array
            std::vector<float> laser_data=msgs->ranges;
            int msg_size=laser_data.size();
            for(int i = 0; i < msg_size; i++){
                if(msgs->ranges[i] < shortestReading && msgs->ranges[i] > 0.1){
                    shortestReading = msgs->ranges[i];
                }
            }

            //Set new shortest distance if shorter than previous
            if(shortestReading<shortestDistance){
                shortestDistance = shortestReading;
            }

            //Print out the currently shortest distance measured
	        myfile.open("/home/ros/Desktop/DistanceMeasurements.txt"); //Change the filepath + name according to needs
	        myfile << "The shortest distance measured in this test:\n";
	        myfile << shortestDistance << " m";
	        myfile.close();
            std::cout << "The shortest distance measured is: " << shortestDistance << " m" << std::endl;
 
        }
};


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "smallest_laser_distance_node");
    calculateDistance calculateDistance_object;
    ROS_INFO("ROSNODE initialised");

    while(ros::ok()){
        ros::spinOnce();
    }

    return 0;
}

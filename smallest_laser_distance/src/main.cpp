#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"


class calculateDistance{
    private:
        ros::Subscriber sub;
        ros::NodeHandle nh;
        float shortestDistance = 100;
        float shortestReading = 100;


    public:
        calculateDistance(){
            ROS_INFO("Object is being created");   
            //Change the topic of the subscriber to fit the RPLidar
            sub = nh.subscribe("scan", 10, &calculateDistance::callback, this);
        }


        void callback(const sensor_msgs::LaserScan::ConstPtr& msgs){
            std::cout << "The shortest distance measured is: " << shortestDistance << " m" << std::endl;
            //Determine the smalles non-zero number in the array
            ROS_INFO("DEBUGGING 1");
            std::vector<float> laser_data=msgs->ranges;
            int msg_size=laser_data.size();
            for(int i = 0; i < msg_size; i++){ //THIS SIZEOF() FUNCTION DOESN'T TAKE IN ALL READINGS, SO THIS WONT WORK PROPERLY!!
                std::cout << "DEBUGGING 2: " << i << "Distance is: " << msgs->ranges[i] << std::endl;
                if(msgs->ranges[i] < shortestReading && msgs->ranges[i] > 0.1){
                    ROS_INFO("DEBUGGING 3");
                    shortestReading = msgs->ranges[i];
                }
            }

            //Set new shortest distance if shorter than previous
            ROS_INFO("DEBUGGING 4");
            if(shortestReading<shortestDistance){
                ROS_INFO("DEBUGGING 5");
                shortestDistance = shortestReading;
            }

            //Print out the currently shortest distance measured
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
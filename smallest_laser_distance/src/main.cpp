#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>


class calculateDistance{
    private:
        ros::Subscriber sub;
        ros::NodeHandle nh;
        float shortestDistance;

    public:
        calculateDistance(){
            ROS_INFO("Object is being created");   
            //Change the topic of the subscriber to fit the RPLidar
            sub = nh.subscribe("laser_front/scan", 10, &calculateDistance::callback, this);
        }


        void callback(sensor_msgs::LaserScan msgs){
            if(msgs.range_min<shortestDistance){
                shortestDistance = msgs.range_min;
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
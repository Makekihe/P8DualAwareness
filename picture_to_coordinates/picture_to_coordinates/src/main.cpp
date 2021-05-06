#include <stdio.h>
#include <stdlib.h>
#include <numeric>
#include <librealsense2/rs.hpp>
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Transform.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"


//#define STREAM          RS2_STREAM_DEPTH  // rs2_stream is a types of data provided by RealSense device           //
#define FORMAT          RS2_FORMAT_Z16    // rs2_format identifies how binary data is encoded within a frame      //
#define WIDTH           1280               // Defines the number of columns for each frame or zero for auto resolve//
#define HEIGHT          720                 // Defines the number of lines for each frame or zero for auto resolve  //
#define FPS             15               // Defines the rate of frames per second                                //
#define STREAM_INDEX    0                 // Defines the stream index, used for multiple streams of the same type //

class doItAll{
    private:
        ros::Publisher pub;
        ros::Subscriber sub;
        ros::NodeHandle nh;
        rs2::pipeline p;
	    rs2::config cfg;
        rs2::threshold_filter threshold_filter;
        //rs2::decimation_filter dec_filter;
        //rs2::spatial_filter spatial_filter;
        //rs2::temporal_filter temp_filter;
        //rs2::hole_filling_filter hole_filter;
        //std::vector<geometry_msgs::Transform> poses;
        float xstart;
        float ystart;
        float xend;
        float yend;
        float id;
        //std::vector<float> coordinates; //XYZ, saved for real world coordinates 
        float cx=635.2405f, cy=359.3353f, fx=637.2601f, fy=637.2601f; // intrinsic for 1280x720
    public:
        doItAll(){
            std::cout << "Object is being created" << std::endl;
            //pub=nh.advertise<std_msgs::Float32MultiArray>("coordinates", 10);
            pub=nh.advertise<geometry_msgs::Pose>("coordinates", 10);
            //sub=nh.subscribe("myBoundingposes", 10, &doItAll::fake_callback, this);
            sub=nh.subscribe("/body_pose", 10, &doItAll::callback, this);
            // STREAM, STREAM_INDEX, WIDTH, HEIGHT, FORMAT, FPS, &e);
            cfg.enable_stream(RS2_STREAM_DEPTH, STREAM_INDEX, WIDTH, HEIGHT, FORMAT, FPS); 
            threshold_filter.set_option(RS2_OPTION_MIN_DISTANCE, 0.1f);
            threshold_filter.set_option(RS2_OPTION_MAX_DISTANCE, 6.5f);
            p.start(cfg);
	        std::cout << "Object is ready" << std::endl;
        }

        void callback(const geometry_msgs::Transform::ConstPtr& msg){
           
            rs2::frameset frames=p.wait_for_frames();
            rs2::depth_frame depth_map=frames.get_depth_frame();

            depth_map=threshold_filter.process(depth_map);

            xstart=msg->rotation.x;
            ystart=msg->rotation.y;
            xend=msg->rotation.z;
            yend=msg->rotation.w;
            id=msg->translation.x;

            geometry_msgs::Pose pose;

            int xmin=xstart;
            int ymin=ystart;
            int xmax=xend-1;
            int ymax=yend-1;
            int xdelta=xend-xstart;
            int ydelta=yend-ystart;
            int center[2];

            int skip = 0;

            if (xdelta==0 || ydelta==0) {
                skip=1;
            }
            
            center[0]=xdelta/2;
            center[1]=ydelta/2;

            std::vector<float> depth_kernel;
            depth_kernel.clear();
            float depth;
            float zcoordinate;
                    

            depth=depth_map.get_distance(xmin+center[0],ymin+center[1]); //get distance in center pixel
            
            if (depth>0.1 && depth<10) { //Valid depth value defined as 0.1<depth<10
                zcoordinate=depth;
            } else {
                skip=1; //If no valid depth value, check next picture instead
            }
              

            float xcoordinate=((center[0]-cx)*depth)/fx; //((u-cx)*Z)/fx;
            float ycoordinate=((center[1]-cy)*depth)/fy; //((v-cy)*Z)/fy;

            pose.position.x = xcoordinate;
            pose.position.y = ycoordinate;
            pose.position.z = zcoordinate;

            if (skip==0) {
                pub.publish(pose);
                //pub.publish(pub_array);
                std::cout << "I have published coordinates." << std::endl; 
            }           
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "doItAll_class");
    doItAll doItAll_object;
    
    while(ros::ok()){
        ros::spinOnce();
        //sleep(1);
    }

return 0;
}
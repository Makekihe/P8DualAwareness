#include <stdio.h>
#include <stdlib.h>
#include <numeric>
#include <librealsense2/rs.hpp>
#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/ObjectCount.h"

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
        std::vector<darknet_ros_msgs::BoundingBox> boxes;
        //std::vector<float> coordinates; //XYZ, saved for real world coordinates 
        float cx=635.2405f, cy=359.3353f, fx=637.2601f, fy=637.2601f; // intrinsic for 1280x720
    public:
        doItAll(){
            std::cout << "Object is being created" << std::endl;
            pub=nh.advertise<std_msgs::Float32MultiArray>("coordinates", 10);;
            //sub=nh.subscribe("myBoundingBoxes", 10, &doItAll::fake_callback, this);
            sub=nh.subscribe("darknet_ros/bounding_boxes", 10, &doItAll::callback, this);
            // STREAM, STREAM_INDEX, WIDTH, HEIGHT, FORMAT, FPS, &e);
            cfg.enable_stream(RS2_STREAM_DEPTH, STREAM_INDEX, WIDTH, HEIGHT, FORMAT, FPS); 
            threshold_filter.set_option(RS2_OPTION_MIN_DISTANCE, 0.1f);
            threshold_filter.set_option(RS2_OPTION_MAX_DISTANCE, 6.5f);
            p.start(cfg);
	        std::cout << "Object is ready" << std::endl;
        }

        void fake_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){

            boxes=msg->bounding_boxes;
            std_msgs::Float32MultiArray pub_array;
            pub_array.data.clear();
            
            for (size_t i = 0; i < boxes.size(); i++){
                if(boxes[i].Class=="human"){
                    int xmin=boxes[i].xmin;
                    int ymin=boxes[i].ymin;
                    int xmax=boxes[i].xmax;
                    int ymax=boxes[i].ymax;
                    int center[2];
                    center[0]=xmax-xmin;
                    center[1]=ymax-ymin;

                    float depth=4.5f;

                    float xcoordinate=((center[0]-cx)*depth)/fx; //((u-cx)*Z)/fx;
                    std::cout<<"xcoordinate "<<xcoordinate<<std::endl;
                    pub_array.data.push_back(xcoordinate);
                    float ycoordinate=((center[1]-cy)*depth)/fy; //((v-cy)*Z)/fy;
                    std::cout<<"ycoordinate "<<ycoordinate<<std::endl;
                    pub_array.data.push_back(ycoordinate);
                    float zcoordinate=depth;
                    std::cout<<"zcoordinate "<<zcoordinate<<std::endl;
                    pub_array.data.push_back(zcoordinate);
                }
            }
            
            pub.publish(pub_array);
            std::cout << "I have published coordinates." << std::endl;

        }

        void callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){
            //std::cout << "Grabbing frames" << std::endl;
            rs2::frameset frames=p.wait_for_frames();
            rs2::depth_frame depth_map=frames.get_depth_frame(); //
            //std::cout << "Filters" << std::endl;
            depth_map=threshold_filter.process(depth_map);
            /*depth_map=dec_filter.process(depth_map);
            depth_map=spatial_filter.process(depth_map);
            depth_map=temp_filter.process(depth_map);
            depth_map=hole_filter.process(depth_map);*/

            boxes.clear();
            boxes=msg->bounding_boxes;
            int seq_id=msg->header.seq;
            std_msgs::Float32MultiArray pub_array;
            pub_array.data.clear();
            //pub_array.data.push_back(seq_id);

            for (size_t i = 0; i < boxes.size(); i++){
                //std::cout << "Waiting in the for loop" << std::endl;
                if(boxes[i].Class=="person"){
                    //std::cout << "Grabbing corners" << std::endl;
                    int xmin=boxes[i].xmin;
                    int ymin=boxes[i].ymin;
                    int xmax=boxes[i].xmax-1;
                    int ymax=boxes[i].ymax-1;
                    int xdelta=xmax-xmin;
                    int ydelta=ymax-ymin;
                    int center[2];
                    center[0]=xdelta/2;
                    center[1]=ydelta/2;
                    //float depth=depth_map.get_distance(center[0],center[1]);

                    std::vector<float> depth_kernel;
                    depth_kernel.clear();
                    float depth;

                    for (size_t i = 0; i < xdelta; i++){                
                        for (size_t j = 0; j < ydelta; j++){
                            depth=depth_map.get_distance(xmin+i,ymin+j);
                            depth_kernel.push_back(depth);
                        }
                    }

                    //calculate the median of the collected depth values
                    std::sort(depth_kernel.begin(),depth_kernel.end());
                    size_t kernel_size = depth_kernel.size();
                    float depth_median;
                    if (kernel_size % 2 == 0){
                        depth_median=depth_kernel[kernel_size/2-1]+depth_kernel[kernel_size/2]/2;
                    } else {
                        depth_median=depth_kernel[kernel_size/2];
                    }
                    std::cout << "Median depth is "<< depth_median << std::endl;
                    depth=depth_median;

                    /*float depth_min=0;
                    for (size_t i = 0; i < depth_kernel.size(); i++)
                    {
                        depth_min=depth_kernel[i];
                        if(depth_min!=0) break;
                    }
                    
                    float depth_min=*std::min_element(depth_kernel.begin(),depth_kernel.end());
                    float depth_avg=std::accumulate(depth_kernel.begin(),depth_kernel.end(),0.0f)/depth_kernel.size();
                    std::cout << "Average depth is "<< depth_avg << std::endl;
                    std::cout << "Minimum depth is "<< depth_min << std::endl;*/

                    float xcoordinate=((center[0]-cx)*depth)/fx; //((u-cx)*Z)/fx;
                    pub_array.data.push_back(xcoordinate);
                    float ycoordinate=((center[1]-cy)*depth)/fy; //((v-cy)*Z)/fy;
                    pub_array.data.push_back(ycoordinate);
                    float zcoordinate=depth;
                    pub_array.data.push_back(zcoordinate);

                    pub.publish(pub_array);
                    std::cout << "I have published coordinates." << std::endl;      
                }
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


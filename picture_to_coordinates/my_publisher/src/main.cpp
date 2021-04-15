#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/ObjectCount.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "boundingBoxesPublisher");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<darknet_ros_msgs::BoundingBoxes>("myBoundingBoxes", 100);
	uint i=0;

	while (ros::ok())
	{
		darknet_ros_msgs::BoundingBoxes pubBoxes;
		pubBoxes.header.seq=i;
		++i;
		pubBoxes.header.stamp=ros::Time::now();
		pubBoxes.header.frame_id="fake_frame";

		std::vector<darknet_ros_msgs::BoundingBox> myBoxes;
		myBoxes.clear();
		myBoxes.resize(2);

		myBoxes[0].xmin=0;
		myBoxes[0].ymin=10;
		myBoxes[0].xmax=20;
		myBoxes[0].ymax=30;
		myBoxes[0].id=0;
		myBoxes[0].probability=0.5;
		myBoxes[0].Class="human";

		myBoxes[1].xmin=40;
		myBoxes[1].ymin=50;
		myBoxes[1].xmax=60;
		myBoxes[1].ymax=70;
		myBoxes[1].id=1;
		myBoxes[1].probability=0.75;
		myBoxes[1].Class="cat";

		pubBoxes.bounding_boxes = myBoxes;

		pub.publish(pubBoxes);
		
		//Let the world know
		ROS_INFO("I published something!");
		//Do this.
		ros::spinOnce();
		//Added a delay so not to spam
		sleep(1);
	}

}
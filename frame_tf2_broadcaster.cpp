#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_broadcaster");
  ros::NodeHandle node;

   tf2_ros::TransformBroadcaster tfb;
  geometry_msgs::TransformStamped transformA;
  geometry_msgs::TransformStamped transformB;
  geometry_msgs::TransformStamped transformC;
  geometry_msgs::TransformStamped transformD;
  geometry_msgs::TransformStamped transformE;

//Baselink to length | under_tower_joint
  transformA.header.frame_id = "base_link";
  transformA.child_frame_id = "under_tower";
  transformA.transform.translation.x = 0.92;
  transformA.transform.translation.y = 0.0;
  transformA.transform.translation.z = 0.0;
  tf2::Quaternion a;
        a.setRPY(0, 0, 0);
  transformA.transform.rotation.x = a.x();
  transformA.transform.rotation.y = a.y();
  transformA.transform.rotation.z = a.z();
  transformA.transform.rotation.w = a.w();


//Length to height | tower_base_joint
  transformB.header.frame_id = "under_tower";
  transformB.child_frame_id = "tower_connection";
  transformB.transform.translation.x = 0.0;
  transformB.transform.translation.y = 0.0;
  transformB.transform.translation.z = 0.477;
  tf2::Quaternion b;
        b.setRPY(0, 0, 0);
  transformB.transform.rotation.x = b.x();
  transformB.transform.rotation.y = b.y();
  transformB.transform.rotation.z = b.z();
  transformB.transform.rotation.w = b.w();


//Tower to camera | camera_joint  
  transformC.header.frame_id = "tower_connection";
  transformC.child_frame_id = "camera_base";
  transformC.transform.translation.x = 0.03315;
  transformC.transform.translation.y = 0.0;
  transformC.transform.translation.z = 0.078;
  tf2::Quaternion c;
        c.setRPY(0, 0, 0);
  transformC.transform.rotation.x = c.x();
  transformC.transform.rotation.y = c.y();
  transformC.transform.rotation.z = c.z();
  transformC.transform.rotation.w = c.w();


//Camera to image | camera_base_to_image
  transformD.header.frame_id = "camera_base";
  transformD.child_frame_id = "camera_colour_frame";
  transformD.transform.translation.x = 0.0106;
  transformD.transform.translation.y = 0.0325;
  transformD.transform.translation.z = 0.0125;
  tf2::Quaternion d;
        d.setRPY(-1.57079, 0, -1.57079);
  transformD.transform.rotation.x = d.x();
  transformD.transform.rotation.y = d.y();
  transformD.transform.rotation.z = d.z();
  transformD.transform.rotation.w = d.w();


//Tower to lidar | lidar_joint
  transformE.header.frame_id = "tower_connection";
  transformE.child_frame_id = "lidar_base";
  transformE.transform.translation.x = 0.01283;
  transformE.transform.translation.y = 0.0;
  transformE.transform.translation.z = 0.1506;
  tf2::Quaternion e;
        e.setRPY(0, 0, 0);
  transformE.transform.rotation.x = e.x();
  transformE.transform.rotation.y = e.y();
  transformE.transform.rotation.z = e.z();
  transformE.transform.rotation.w = e.w();


  ros::Rate rate(10000.0);
  while (node.ok()){
    transformA.header.stamp = ros::Time::now();
    tfb.sendTransform(transformA);
    //rate.sleep();
    printf("sending A\n");

    transformB.header.stamp = ros::Time::now();
    tfb.sendTransform(transformB);
    //rate.sleep();
    printf("sending B\n");

    transformC.header.stamp = ros::Time::now();
    tfb.sendTransform(transformC);
    //rate.sleep();
    printf("sending C\n");

    transformD.header.stamp = ros::Time::now();
    tfb.sendTransform(transformD);
    //rate.sleep();
    printf("sending D\n");

    transformE.header.stamp = ros::Time::now();
    tfb.sendTransform(transformE);
    //
    printf("sending E\n");
    rate.sleep();
  }

};

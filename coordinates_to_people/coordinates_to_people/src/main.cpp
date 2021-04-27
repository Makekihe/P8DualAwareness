#include <ros/ros.h>
#include <people_msgs/People.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>


class calculateVelocity{
    private:
        ros::Publisher pub;
        ros::Subscriber sub;
        ros::NodeHandle nh;
        people_msgs::People people;
        people_msgs::Person person;
        float previousCoordinates [3] = {0, 0, 0};          //TODO: Figure out how to handle the first time a detection is made
        ros::Time startTime;
        ros::Time stopTime;
        //ros::Duration fiveSeconds(5.0);

    public: 
        calculateVelocity(){
            ROS_INFO("Object is being created");
            pub = nh.advertise<people_msgs::People>("people",10);
            sub = nh.subscribe("coordinates", 10, &calculateVelocity::callback, this);
        }


        void callback(geometry_msgs::PoseArray msgs){

            startTime = ros::Time::now();
            double timestep = 0.01;                             //TODO: Calculate the timestep
            float headingVector [3];
            float futureCoordinates[3];
            float currentCoordinates [3];
            //float currentCoordinates [3] = {4, 3, 2};
                                                                //TODO: Figure out how to handle multiple people (not necessary for now)
            

            //The following is with the intention of trying to handle the first time a detection is made.
            //My logic would be to determine if a certain longer timespan has passed between two detections, to determine if the new detection
            //should be connected to the previous detection, if it should a "new first detection" for a new person. These start/stop-Time would
            //then be in the start and end of the callback function, and we could compare the times - doesn't work atm.
            //Feel free to take another approach.
            // Potential help: http://wiki.ros.org/roscpp/Overview/Time
            // OR: https://answers.ros.org/question/276110/typeerror-cannot-compare-to-non-duration/
            //if ((startTime-stopTime) < fiveSeconds){

            //}
            
            //Assign coordinates to array
            currentCoordinates[0] = msgs.poses[0].position.x;
            currentCoordinates[1] = msgs.poses[0].position.y;
            currentCoordinates[2] = msgs.poses[0].position.z;

            std::cout << "Coordinates of the current position is: " << std::endl;
            std::cout << "X: "<< currentCoordinates[0] << std::endl;
            std::cout << "Y: "<< currentCoordinates[1] << std::endl;
            std::cout << "Z: "<< currentCoordinates[2] << std::endl;


            //Calculate the heading vector (t-1 to t)
            headingVector[0] = currentCoordinates[0] - previousCoordinates[0];
            headingVector[1] = currentCoordinates[1] - previousCoordinates[1];
            headingVector[2] = currentCoordinates[2] - previousCoordinates[2];

            std::cout << "Coordinates of the heading vector is: " << std::endl;
            std::cout << "X: "<< headingVector[0] << std::endl;
            std::cout << "Y: "<< headingVector[1] << std::endl;
            std::cout << "Z: "<< headingVector[2] << std::endl;


            //Predic future pose (t+1) using heading vector
            futureCoordinates[0] = currentCoordinates[0] + headingVector[0];
            futureCoordinates[1] = currentCoordinates[1] + headingVector[1];
            futureCoordinates[2] = currentCoordinates[2] + headingVector[2];

            std::cout << "Coordinates of the future position is: " << std::endl;
            std::cout << "X: "<< futureCoordinates[0] << std::endl;
            std::cout << "Y: "<< futureCoordinates[1] << std::endl;
            std::cout << "Z: "<< futureCoordinates[2] << std::endl;


            //Calculate the linear velocity from t-1 to t
            float velocity = (sqrt(pow(headingVector[0], 2) + pow(headingVector[1], 2) + pow(0, 2))) / timestep;
            std::cout << "The linear velocity of the human is: "<< velocity << "m/s" << std::endl;


            //Define the people msgs
            person.name = "Oskar";                              //TODO: Figure out how to assign different names to different persons (not necessary for now)
            person.position.x = currentCoordinates[0];
            person.position.y = currentCoordinates[1];
            person.position.z = 0; //set to zero as we publish people on the ground
            person.velocity.x = futureCoordinates[0];
            person.velocity.y = futureCoordinates[1];
            person.velocity.z = 0; //set to zero as we publish people on the ground
            //person.reliability = 0.5;

            people.header = msgs.header;
            //If the header stamp is empty, then assign the current time
            if (people.header.stamp == ros::Time(0)){
                people.header.stamp = ros::Time::now();
            }

            //Push this person in the people array
            people.people.push_back(person);
            
            
            //Publish the information
            std::cout << "Publishing people msgs" << std::endl;
            pub.publish(people);

            //Assign the current coordinates to previous coordinates for next time the callback is run
            previousCoordinates[0] = currentCoordinates[0];
            previousCoordinates[1] = currentCoordinates[1];
            previousCoordinates[2] = currentCoordinates[2];
            stopTime = ros::Time::now();

        }



};





int main(int argc, char* argv[])
{
    ros::init(argc, argv, "coordinates_to_people_node");
    calculateVelocity calculateVelocity_object;
    ROS_INFO("ROSNODE initialised");

    while(ros::ok()){
        ros::spinOnce();
    }

    return 0;
}
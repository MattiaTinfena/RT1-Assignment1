#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/Kill.h" 
#include <iostream>
#include <string>
#include <string.h>

ros::Publisher pub;
ros::Subscriber sub;

void turtleCallback(const turtlesim::Pose::ConstPtr& msg, int* vel)
	{
	ROS_INFO("Turtle subscriber@[%f, %f, %f]",  
	msg->x, msg->y, msg->theta);
	geometry_msgs::Twist my_vel;
    my_vel.linear.x = vel[0];
    my_vel.linear.y = vel[1];
    my_vel.angular.z = vel[1];
    pub.publish(my_vel);
}



int main (int argc, char **argv)
{
    std:: string choice;
    int speed[3];

// Initialize the node, setup the NodeHandle for handling the communication with the ROS //system  
	ros::init(argc, argv, "turtlebot_subscriber");  
	ros::NodeHandle nh;

	ros::ServiceClient client1 =  nh.serviceClient<turtlesim::Spawn>("/spawn");

    //Spawning a new turtle
	turtlesim::Spawn srv1;
	srv1.request.x = 2.0;  
	srv1.request.y = 1.0;
	srv1.request.theta = 0.0;
	srv1.request.name = "turtle2";
	client1.call(srv1);
	
    while(ros::ok){
        std::cout << "Choose a turtle:" << std::endl;
        std::cout << "1. turtle 1" << std::endl;
        std::cout << "2. turtle 2" << std::endl;
        std::cout << "q. quit" << std::endl;
        std::cin >> choice;

        if(choice == "q"){
                break;
            }

        while(choice != "1" && choice != "2"){
            std::cout << "Invalid choice, choose 1,2 or q" << std::endl;
            std::cin >> choice;
            if(choice == "q"){
                return 0;
            }
        }
        std::string turtle = choice == "1" ? "turtle1" : "turtle2";
        std::cout << "Set a speed along x,y,theta" << std::endl;
        std::cin >> speed[0] >> speed[1] >> speed[2];

        pub = nh.advertise<geometry_msgs::Twist>(turtle + "/cmd_vel",1);  //check the topic name
        sub = nh.subscribe<turtlesim::Pose>(turtle +"/pose", 1, boost::bind(turtleCallback, _1, speed));  //check the topic name

        sleep(1);
        ros::spinOnce();
    }
	return 0;
}

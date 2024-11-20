#include <iostream>
#include <string>
#include <string.h>
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/Kill.h" 

ros::Publisher pub1,pub2;
ros::Subscriber sub;
std::string turtle;

void turtleSetSpeed(int* vel){
    // Set the turtle's speed
	geometry_msgs::Twist my_vel;
    my_vel.linear.x = vel[0];
    my_vel.linear.y = vel[1];
    my_vel.angular.z = vel[2];

    if(turtle == "turtle1"){
        pub1.publish(my_vel);
    }else{
        pub2.publish(my_vel);
    }
}


int main (int argc, char **argv)
{
    // Ros init
	ros::init(argc, argv, "turtlebotUInode");  
	ros::NodeHandle nh;

    // set up the publisher
	ros::ServiceClient client1 =  nh.serviceClient<turtlesim::Spawn>("/spawn");
    pub1 = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1); 
    pub2 = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel",1);  
    
    std:: string choice;
    int speed[3];

    //Spawning a new turtle
	turtlesim::Spawn srv1;
	srv1.request.x = 3.0;  
	srv1.request.y = 3.0;
	srv1.request.theta = 0.0;
	srv1.request.name = "turtle2";
	client1.call(srv1);
	
    while(ros::ok){
        // Print the main menu
        std::cout << "Choose a turtle:" << std::endl;
        std::cout << "1. turtle 1" << std::endl;
        std::cout << "2. turtle 2" << std::endl;
        std::cout << "q. quit" << std::endl;
        std::cin >> choice;

        // q to quit
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

        // Read the speeds
        turtle = choice == "1" ? "turtle1" : "turtle2";
        std::cout << "Set a speed along x" << std::endl;
        std::cin >> speed[0];
        std::cout << "Set a speed along y" << std::endl;
        std::cin >> speed[1];
        std::cout << "Set a speed along theta" << std::endl;
        std::cin >> speed[2];

        int duration = 10;  // Make the turtle move for 1 second
        ros::Rate rate(10);  // Set the loop rate in Hz

        while(ros::ok && duration > 0){
            if(duration == 10){
                turtleSetSpeed(speed);
            }
            duration--;
            ros::spinOnce();
            rate.sleep();
        }

        // Reset the speed
        speed[0] = 0;
        speed[1] = 0;
        speed[2] = 0;

        turtleSetSpeed(speed);

        ros::spinOnce();
    }
	return 0;
}

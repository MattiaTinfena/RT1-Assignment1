#include <iostream>
#include <string>
#include <string.h>
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/Kill.h" 

int main (int argc, char **argv)
{
    // Ros init
	ros::init(argc, argv, "turtlebotTresholdnode");  
	ros::NodeHandle nh;
    
    std:: string choice;

	
    while(ros::ok()){
        // Print the main menu
        std::cout << "Choose a tresholdbetween 1 and 8 or q:" << std::endl;
        std::cin >> choice;

        // q to quit
        if(choice == "q"){
                break;
            }
        
        int treshold;

        try{
            treshold = std::stoi(choice);
        } 

        while(treshold < 1 || treshold > 8){
            std::cout << "Invalid choice, choose a treshold between 1 qnd 8 or q:" << std::endl;
            std::cin >> choice;
            if(choice == "q"){
                return 0;
            }
            try{
                treshold = std::stoi(choice);
            }
            catch (const std::exception &e){
                std::cout << "Invalid input. Please enter a number between 1 and 8 or 'q' to quit." << std::endl;
                continue;
            }
        }

        ros::param::set("/tresholdParam", treshold);

        ros::spinOnce();
    }
	return 0;
}

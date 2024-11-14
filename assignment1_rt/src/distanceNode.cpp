#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include <cmath>
#include "std_msgs/Float32.h"

ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher distance_pub;
geometry_msgs::Twist stop_msg;

float   xt1[2] = {0,0},
        yt1[2] = {0,0},
        xt2[2] = {0,0}, 
        yt2[2] = {0,0};

float   distance[2] = {0,0};

bool    distanceFromBorders1 = false;
bool    distanceFromBorders2 = false;
bool    stop = false;

// This function stops the turtle if the distance between them is less than 2.
void stopHandle(){
    if(stop){
        stop = false;

        // Stop turtle1
        pub1.publish(stop_msg);

        // Stop turtle2
        pub2.publish(stop_msg);
    }
    if(distanceFromBorders1){
        distanceFromBorders1 = false;
        // Stop turtle1
        pub1.publish(stop_msg);
    }
    if(distanceFromBorders2){
        distanceFromBorders2 = false;
        // Stop turtle2
        pub2.publish(stop_msg);
    }
}

// Callback to update Turtle 1's position
void turtlePosition1(const turtlesim::Pose::ConstPtr& msg){
    ROS_INFO("Turtle 1 Position: [%f, %f, %f]", 
        msg->x, msg->y, msg->theta);
    xt1[1] = xt1[0];
    yt1[1] = yt1[0];
    xt1[0] = msg->x;
    yt1[0] = msg->y;
}

// Callback to update Turtle 2's position
void turtlePosition2(const turtlesim::Pose::ConstPtr& msg){
    ROS_INFO("Turtle 2 Position: [%f, %f, %f]", 
        msg->x, msg->y, msg->theta);
    xt2[1] = xt2[0];
    yt2[1] = yt2[0];
    xt2[0] = msg->x;
    yt2[0] = msg->y;
}

int main (int argc, char **argv){
    ros::init(argc, argv, "turtleDistanceNode"); 
  
    ros::NodeHandle nh;

    stop_msg.linear.x = 0;
    stop_msg.linear.y = 0;
    stop_msg.angular.z = 0;

    // Publishers for the turtles
    pub1 = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    pub2 = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 1);
    
    distance_pub = nh.advertise<std_msgs::Float32>("turtle_distance", 1);

    // Subscribers for the turtles' positions
    ros::Subscriber sub1 = nh.subscribe<turtlesim::Pose>("turtle1/pose", 1, turtlePosition1);
    ros::Subscriber sub2 = nh.subscribe<turtlesim::Pose>("turtle2/pose", 1, turtlePosition2);

    ros::Rate rate(10);  // Set the loop rate in Hz

    while(ros::ok()){
        distance[1] = distance[0];
        // Calculate the distance between the turtles
        distance[0] = sqrt(pow(xt1[0] - xt2[0], 2) + pow(yt1[0] - yt2[0], 2));

        // Publish the distance
        std_msgs::Float32 distance_msg;
        distance_msg.data = distance[0];
        distance_pub.publish(distance_msg);

        if(distance[0] < 2 && distance[0] < distance[1]){
            stop = true;
        }

        if( (xt1[0] >= 8 && (xt1[0] > xt1[1])) || 
            (yt1[0] >= 8 && (yt1[0] > yt1[1])) || 
            (xt1[0] <= 2 && (xt1[0] < xt1[1])) || 
            (yt1[0] <= 2 && (yt1[0] < yt1[1]))){

            distanceFromBorders1 = true;
        }

        if( (xt2[0] >= 8 && (xt2[0] > xt2[1])) || 
            (yt2[0] >= 8 && (yt2[0] > yt2[1])) || 
            (xt2[0] <= 2 && (xt2[0] < xt2[1])) || 
            (yt2[0] <= 2 && (yt2[0] < yt2[1]))){
            
            distanceFromBorders2 = true;
        }

        // Call function to stop turtles if they are too close or if they are hitting the walls
        stopHandle();

        ros::spinOnce();  // Handle callback functions
        rate.sleep();     // Sleep for the remainder of the loop cycle
    }
    return 0;
}


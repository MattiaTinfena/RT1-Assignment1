#include <cmath>
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"

ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher distance_pub;
geometry_msgs::Twist stop_msg;

// Are saved also the previous distance and the previous position in order to 
// unlock the turtles if they want ot make an allowed movement 

float   xt1[2] = {0,0},
        yt1[2] = {0,0},
        xt2[2] = {0,0}, 
        yt2[2] = {0,0};

float   distance[2] = {0,0};

float   safePos1[2] = {3,3};
float   safePos2[2] = {7,7};

int     safesel[2] = {0,0};

bool    distanceFromBorders1 = false;
bool    distanceFromBorders2 = false;
bool    stop = false;

float computeDistance( float x1, float y1, float x2, float y2){
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}
void stopHandle(){
    if(stop){
        ROS_INFO("Stop the turtles, they are collinding");
        stop = false;
        // Stop both turtles
        pub1.publish(stop_msg);
        pub2.publish(stop_msg);
    }
    if(distanceFromBorders1){
        ROS_INFO("Stop turtle 1, collision against the border");
        distanceFromBorders1 = false;
        // Stop turtle1
        pub1.publish(stop_msg);
    }
    if(distanceFromBorders2){
        ROS_INFO("Stop turtle 1, collision against the border");
        distanceFromBorders2 = false;
        // Stop turtle2
        pub2.publish(stop_msg);
    }
}

// Callback to update Turtle 1's position
void turtlePosition1(const turtlesim::Pose::ConstPtr& msg){
    xt1[1] = xt1[0];
    yt1[1] = yt1[0];
    xt1[0] = msg->x;
    yt1[0] = msg->y;
}

// Callback to update Turtle 2's position
void turtlePosition2(const turtlesim::Pose::ConstPtr& msg){
    xt2[1] = xt2[0];
    yt2[1] = yt2[0];
    xt2[0] = msg->x;
    yt2[0] = msg->y;
}

int findSafePosition(){
    
    if(stop){
    ROS_INFO("Stop the turtles, they are collinding");
    stop = false;
    return 3;
    }

    if(distanceFromBorders1){
        ROS_INFO("Move turtle 1 to safe place");
        distanceFromBorders1 = false;
        if(computeDistance(xt1[0], yt1[0], safePos1[0], safePos1[1]) < computeDistance(xt1[0], yt1[0], safePos2[0], safePos2[1])){
            // turtle 1 nearest to safe position 1
            safesel = 1;
            return 1;
        }
        else{
            // turtle 1 nerest to safe position 2
            safesel = 2;
            return 1;
        }
    }
    if(distanceFromBorders2){
        ROS_INFO("Move turtle 2 to safe place");
        distanceFromBorders2 = false;

        if(computeDistance(xt2[0], yt2[0], safePos1[0], safePos1[1]) < computeDistance(xt2[0], yt2[0], safePos2[0], safePos2[1])){
            // turtle 2 nearest to safe position 1
            safesel= 1;
            return 2;
        }
        else{
            // turtle 2 nearest to safe position 2
            safesel = 2;
            return 2;
        }
    }
    
}

void movetoSafePosition(int i ){
    
    float directions [2];
    geometry_msgs::Twist movetosafe;

    if(i == 1){
        //send turtle 1 to nearest safe direction
        if(safesel == 1){
            directions[0] = (int)(xt1[0] - safePos1[0]);
            directions[1] = (int)(yt1[0] - safePos1[1]);
        } else{
            directions[0] = (int)(xt1[0] - safePos2[0]);
            directions[1] = (int)(yt1[0] - safePos2[1]);
        }
    } else if(i == 2){ 
        //send turtle 2 to nearest safe direction
        if(safesel == 1){
            directions[0] = (int)(xt2[0] - safePos1[0]);
            directions[1] = (int)(yt2[0] - safePos1[1]);
        } else{
            directions[0] = (int)(xt2[0] - safePos2[0]);
            directions[1] = (int)(yt2[0] - safePos2[1]);
        }
    } else{
        //Turtle 1 to safe direction 1
        directions[0] = (int)(xt1[0] - safePos1[0]);
        directions[1] = (int)(yt1[0] - safePos1[1]);
        
        movetosafe.linear.x = directions[0];
        movetosafe.linear.y = directions[1];
        movetosafe.angular.z = 0;

        pub1.publish(movetosafe);

        //Turtle 2 to safe direction 2
        directions[0] = (int)(xt2[0] - safePos2[0]);   
        directions[1] = (int)(yt2[0] - safePos2[1]);

        movetosafe.linear.x = directions[0];
        movetosafe.linear.y = directions[1];
        movetosafe.angular.z = 0;
        pub2.publish(movetosafe);
        return
    }
    
    
    movetosafe.linear.x = directions[0];
    movetosafe.linear.y = directions[1];
    movetosafe.angular.z = 0;

    if(i == 1){
        pub1.publish(movetosafe);
    }
    else{
        pub2.publish(movetosafe);
    }

}

int main (int argc, char **argv){
    // Ros init
    ros::init(argc, argv, "turtlebotDistanceNode"); 
    ros::NodeHandle nh;

    // Set up the speed to stop the turtles
    stop_msg.linear.x = 0;
    stop_msg.linear.y = 0;
    stop_msg.angular.z = 0;

    // Publishers for the turtles
    pub1 = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    pub2 = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 1);

    // Subscribers for the turtles' positions
    ros::Subscriber sub1 = nh.subscribe<turtlesim::Pose>("turtle1/pose", 1, turtlePosition1);
    ros::Subscriber sub2 = nh.subscribe<turtlesim::Pose>("turtle2/pose", 1, turtlePosition2);

    // Publisher for the distance topic
    distance_pub = nh.advertise<std_msgs::Float32>("turtle_distance", 1);
    std_msgs::Float32 distance_msg;

    ros::Rate rate(10);  // Set the loop rate in Hz

    while(ros::ok()){

        // Update the previous position
        distance[1] = distance[0];

        // Compute the distance between the turtles
        distance[0] = sqrt(pow(xt1[0] - xt2[0], 2) + pow(yt1[0] - yt2[0], 2));

        // Publish the distance
        distance_msg.data = distance[0];
        distance_pub.publish(distance_msg);

        // Check if the turtles are colliding
        if(distance[0] < 2 && distance[0] < distance[1]){
            stop = true;
        }

        // Check if the turtle1 is colliding against the borders 
        if( (xt1[0] >= 8 && (xt1[0] > xt1[1])) || 
            (yt1[0] >= 8 && (yt1[0] > yt1[1])) || 
            (xt1[0] <= 2 && (xt1[0] < xt1[1])) || 
            (yt1[0] <= 2 && (yt1[0] < yt1[1]))){

            distanceFromBorders1 = true;
        }
        // Check if the turtle2 is colliding against the borders
        if( (xt2[0] >= 8 && (xt2[0] > xt2[1])) || 
            (yt2[0] >= 8 && (yt2[0] > yt2[1])) || 
            (xt2[0] <= 2 && (xt2[0] < xt2[1])) || 
            (yt2[0] <= 2 && (yt2[0] < yt2[1]))){
            
            distanceFromBorders2 = true;
        }

        // Call function to stop turtles if they are too close or if they are hitting the borders
        //stopHandle();
        int coll = findSafePosition();
        if(coll ==1){
            movetoSafePosition(coll);
        }
        else if(coll == 2){
            movetoSafePosition(coll);
        }
        else if(coll == 3){
            movetoSafePosition(coll);
        }
        ros::spinOnce();  // Handle callback functions
        rate.sleep();     // Sleep for the remainder of the loop cycle
    }
    return 0;
}


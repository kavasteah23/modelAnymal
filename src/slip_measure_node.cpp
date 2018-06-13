#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Vector3.h"
#include "visualization_msgs/MarkerArray.h"
#include <iostream>
#include <math.h>

/* Global variables */
geometry_msgs::Vector3 act_slip[4]; // Storage for every slip in legs
geometry_msgs::Vector3 on_ground_pos[4]; // Storage for old positions of legs
geometry_msgs::Vector3 act_pos[4];

ros::Publisher pubLeg0,pubLeg1,pubLeg2,pubLeg3;
/* Function for calculating length of force vector */
float forceDist(geometry_msgs::Vector3 f){ 
    return std::sqrt(f.x*f.x+f.y*f.y+f.z*f.z);
}
/* Function callback for position of legs topic */
void positionCallback(const visualization_msgs::MarkerArray& msg){
    for (int i = 0;i<4;i++){
        act_pos[i].x = msg.markers[i].pose.position.x;
        act_pos[i].y = msg.markers[i].pose.position.y;
        act_pos[i].z = msg.markers[i].pose.position.z;        
    }
} // Feet count: 0 - LF,1 - RF,2 - LH,3 -RH
/* Function callback for leg 0 forces topic (very similiar four callbacks) */
void forceLFCallback(const geometry_msgs::WrenchStamped& msg){
    static bool toggleState = false;
    // Check if force is very small; if true: foot is in the air
    if (forceDist(msg.wrench.force)<0.01){
        // foot is in the air
        if (toggleState==true)
        {
            toggleState = false;
            act_slip[0].x = std::abs(on_ground_pos[0].x-act_pos[0].x);
            act_slip[0].y = std::abs(on_ground_pos[0].y-act_pos[0].y);
            act_slip[0].z = std::abs(on_ground_pos[0].z-act_pos[0].z);
            pubLeg0.publish(act_slip[0]);
        }
    }
    else{
        if (toggleState==false){
            toggleState = true;
            on_ground_pos[0].x = act_pos[0].x;
            on_ground_pos[0].y = act_pos[0].y;
            on_ground_pos[0].z = act_pos[0].z;
        }             
    }
}
/* Function callback for leg 1 forces topic */
void forceRFCallback(const geometry_msgs::WrenchStamped& msg){
    static bool toggleState = false;
    // Check if force is very small; if true: foot is in the air
    if (forceDist(msg.wrench.force)<0.01){
        // foot is in the air
        if (toggleState==true)
        {
            toggleState = false;
            act_slip[1].x = std::abs(on_ground_pos[1].x-act_pos[1].x);
            act_slip[1].y = std::abs(on_ground_pos[1].y-act_pos[1].y);
            act_slip[1].z = std::abs(on_ground_pos[1].z-act_pos[1].z);
            pubLeg1.publish(act_slip[1]);
        }
    }
    else{
        if (toggleState==false){
            toggleState = true;
            on_ground_pos[1].x = act_pos[1].x;
            on_ground_pos[1].y = act_pos[1].y;
            on_ground_pos[1].z = act_pos[1].z;
        }             
    }   
}
/* Function callback for leg 2 forces topic */
void forceLHCallback(const geometry_msgs::WrenchStamped& msg){
    static bool toggleState = false;
    // Check if force is very small; if true: foot is in the air
    if (forceDist(msg.wrench.force)<0.01){
        // foot is in the air
        if (toggleState==true)
        {
            toggleState = false;
            act_slip[2].x = std::abs(on_ground_pos[2].x-act_pos[2].x);
            act_slip[2].y = std::abs(on_ground_pos[2].y-act_pos[2].y);
            act_slip[2].z = std::abs(on_ground_pos[2].z-act_pos[2].z);
            pubLeg2.publish(act_slip[2]);
        }
    }
    else{
        if (toggleState==false){
            toggleState = true;
            on_ground_pos[2].x = act_pos[2].x;
            on_ground_pos[2].y = act_pos[2].y;
            on_ground_pos[2].z = act_pos[2].z;
        }             
    }    
}
/* Function callback for leg 3 forces topic */
void forceRHCallback(const geometry_msgs::WrenchStamped& msg){
    static bool toggleState = false;
    // Check if force is very small; if true: foot is in the air
    if (forceDist(msg.wrench.force)<0.01){
        // foot is in the air
        if (toggleState==true)
        {
            toggleState = false;
            act_slip[3].x = std::abs(on_ground_pos[3].x-act_pos[3].x);
            act_slip[3].y = std::abs(on_ground_pos[3].y-act_pos[3].y);
            act_slip[3].z = std::abs(on_ground_pos[3].z-act_pos[3].z);
            pubLeg3.publish(act_slip[3]);
        }
    }
    else{
        if (toggleState==false){
            toggleState = true;
            on_ground_pos[3].x = act_pos[3].x;
            on_ground_pos[3].y = act_pos[3].y;
            on_ground_pos[3].z = act_pos[3].z;
        }             
    }    
}
/* Main loop of node */
int main(int argc, char **argv){
    // Initialize node
    ros::init(argc,argv,"slip_measure");
    // Create node handler
    ros::NodeHandle n;
    // Publish to specific topics
    pubLeg0 = n.advertise<geometry_msgs::Vector3>("slip_leg0",1000);
    pubLeg1 = n.advertise<geometry_msgs::Vector3>("slip_leg1",1000);
    pubLeg2 = n.advertise<geometry_msgs::Vector3>("slip_leg2",1000);
    pubLeg3 = n.advertise<geometry_msgs::Vector3>("slip_leg3",1000); 
    // Subscribe from specific topics
    ros::Subscriber posSub = n.subscribe("/loco_ros/measured_foot_positions",1000,positionCallback);
    ros::Subscriber forceLFSub = n.subscribe("/sensors/contact_force_lf_foot_throttle",1000,forceLFCallback);
    ros::Subscriber forceRFSub = n.subscribe("/sensors/contact_force_rf_foot_throttle",1000,forceRFCallback);
    ros::Subscriber forceLHSub = n.subscribe("/sensors/contact_force_lh_foot_throttle",1000,forceLHCallback);
    ros::Subscriber forceRHSub = n.subscribe("/sensors/contact_force_rh_foot_throttle",1000,forceRHCallback);  
    // Entering a loop, pumping callbacks
    ros::spin();

    return 0;
}
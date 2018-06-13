#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Vector3.h"
#include "visualization_msgs/MarkerArray.h"
#include <iostream>
#include <math.h>

/* Global variables */
geometry_msgs::Vector3 forces[4]; // Storage for actual every leg forces
geometry_msgs::Vector3 act_slip[4]; // Storage for every slip in legs
geometry_msgs::Vector3 temps[4]; // Storage for old positions of legs
bool legOnGround[4]; // Storage for actual states of legs - true: on the ground, false: in the air
ros::Publisher pubLeg0,pubLeg1,pubLeg2,pubLeg3;
/* Function for calculating length of force vector */
float forceDist(geometry_msgs::Vector3 f){ 
    return std::sqrt(f.x*f.x+f.y*f.y+f.z*f.z);
}
/* Function callback for position of legs topic */
void positionCallback(const visualization_msgs::MarkerArray& msg){
    for (int i = 0;i<4;i++){ // Go through every leg of robot
        if (legOnGround[i]){
            // Sum previous value of slip and calculate next until Leg is on the ground
            act_slip[i].x += std::abs(msg.markers[i].pose.position.x - temps[i].x);
            act_slip[i].y += std::abs(msg.markers[i].pose.position.y - temps[i].y);
            act_slip[i].z += std::abs(msg.markers[i].pose.position.z - temps[i].z);
        }
        else
        {
            // Reset values of act_slip - Leg is in the air
            act_slip[i].x = 0.0f;
            act_slip[i].y = 0.0f;
            act_slip[i].z = 0.0f;                
        }
        // Save old position of legs
        temps[i].x = msg.markers[i].pose.position.x; 
        temps[i].y = msg.markers[i].pose.position.y; 
        temps[i].z = msg.markers[i].pose.position.z;   
    }
    // Publish calculated actual slips to seperate topics
    pubLeg0.publish(act_slip[0]);
    pubLeg1.publish(act_slip[1]);
    pubLeg2.publish(act_slip[2]);
    pubLeg3.publish(act_slip[3]);
} //Leg count: 0 - LF,1 - RF,2 - LH,3 -RH
/* Function callback for leg 0 forces topic (very similiar four callbacks) */
void forceLFCallback(const geometry_msgs::WrenchStamped& msg){
    // Store on callback values of leg forces
    forces[0].x = msg.wrench.force.x;
    forces[0].y = msg.wrench.force.y;
    forces[0].z = msg.wrench.force.z;
    // Check if force is very small; if true: leg is in the air
    if (forceDist(forces[0])<0.01){
        legOnGround[0] = false;            
    }
    else{
        legOnGround[0] = true;       
    }
}
/* Function callback for leg 1 forces topic */
void forceRFCallback(const geometry_msgs::WrenchStamped& msg){
    forces[1].x = msg.wrench.force.x;
    forces[1].y = msg.wrench.force.y;
    forces[1].z = msg.wrench.force.z;
    if (forceDist(forces[1])<0.01){
        legOnGround[1] = false;            
    }
    else{
        legOnGround[1] = true;       
    }    
}
/* Function callback for leg 2 forces topic */
void forceLHCallback(const geometry_msgs::WrenchStamped& msg){
    forces[2].x = msg.wrench.force.x;
    forces[2].y = msg.wrench.force.y;
    forces[2].z = msg.wrench.force.z;
    if (forceDist(forces[2])<0.01){
        legOnGround[2] = false;            
    }
    else{
        legOnGround[2] = true;       
    }    
}
/* Function callback for leg 3 forces topic */
void forceRHCallback(const geometry_msgs::WrenchStamped& msg){
    forces[3].x = msg.wrench.force.x;
    forces[3].y = msg.wrench.force.y;
    forces[3].z = msg.wrench.force.z;
    if (forceDist(forces[3])<0.01){
        legOnGround[3] = false;            
    }
    else{
        legOnGround[3] = true;       
    }    
}
/* Main loop of node */
int main(int argc, char **argv){
    // Initialization of data - reset every slip values
    for (int i = 0;i<4;i++){
        act_slip[i].x = 0.0f;
        act_slip[i].y = 0.0f;
        act_slip[i].z = 0.0f;    
    }
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
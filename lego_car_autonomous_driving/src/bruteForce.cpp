#include "../include/bruteForce.h"
#include <iostream>


BruteForceDriver::BruteForceDriver(ros::NodeHandle *n)
{

    /*
     Subscriber and Publisher for the BruteForce Driver. We do receive the distance to the next object in front of us.
     Send the Engine Power
    */
    this->mvSubscriber = n->subscribe("UltraSonicRange",100, &BruteForceDriver::receiveMessage, this);
    this->mvPublisher = n->advertise<lego_car_msgs::EnginePower>("enginePowerControl", 100);
    
    // Define the loop rate for the publisher.
    mvLoopRate = new ros::Rate(10);
    this->mvDistance = 0;


    ROS_INFO("BruteForce Driver started");
}

BruteForceDriver::~BruteForceDriver()
{

}

void BruteForceDriver::receiveMessage(const sensor_msgs::Range::ConstPtr& msg)
{
    this->mvDistance = msg->range;
    
}

void BruteForceDriver::run()
{

    
    while (ros::ok())
    {
        ros::spinOnce();
        lego_car_msgs::EnginePower msg;
        msg.mode = lego_car_msgs::EnginePower::TANKMODE;
        if (this->mvDistance < 0.2)
        {
            msg.left_engine = -100;
            msg.right_engine = 100;
        } else
        {
            msg.left_engine = 100;
            msg.right_engine = 100;
        }
        this->mvPublisher.publish(msg);
        
        mvLoopRate->sleep();
        
    }
    
    ROS_INFO("Closed");
}
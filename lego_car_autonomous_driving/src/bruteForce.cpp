#include "../include/bruteForce.h"
#include <iostream>
#include <signal.h>

bool BruteForceDriver::mvRunning = false;


BruteForceDriver::BruteForceDriver(ros::NodeHandle *n)
{

    /*
     Subscriber and Publisher for the BruteForce Driver. We do receive the distance to the next object in front of us.
     Send the Engine Power
    */
    this->mvSubscriber = n->subscribe("UltraSonicRange",100, &BruteForceDriver::receiveMessage, this);
    this->mvPublisher = n->advertise<lego_car_msgs::EnginePower>("enginePowerControl", 100);
    
    this->mvRunning = true;
    signal(SIGINT, BruteForceDriver::shutdown);

    // Define the loop rate for the publisher.
    mvLoopRate = new ros::Rate(10);
    this->mvDistance = 0;


    ROS_INFO("BruteForce Driver started");
}

void BruteForceDriver::shutdown(int sig)
{
    // lego_car_msgs::EnginePower msg;
    // msg.mode = lego_car_msgs::EnginePower::TANKMODE;
    // msg.left_engine = 0;
    // msg.right_engine = 0;
    // mvPublisher.publish(msg);
    mvRunning = false;
    //ros::Duration(0.5).sleep();
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

    
    while (ros::ok() && this->mvRunning)
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
    
    
    lego_car_msgs::EnginePower msg;
    msg.mode = lego_car_msgs::EnginePower::TANKMODE;
    msg.left_engine = 0;
    msg.right_engine = 0;
    for (size_t i = 0; i < 5; i++)
    {
        mvLoopRate->sleep();
        this->mvPublisher.publish(msg);
    }
    
    

    ROS_INFO("Closed");
}
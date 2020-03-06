#include "../include/bruteForce.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Driver");
    //BruteForceDriver driver(argc, argv);
    ros::NodeHandle n;
    BruteForceDriver driver(&n);
    //ros::Subscriber sub = n.subscribe("UltraSonicRange",100, &BruteForceDriver::receiveMessage, &driver);
    
    driver.run();
    //ROS_INFO("Start spinning");
    //ros::spin();
    //return brutForce->run();
}

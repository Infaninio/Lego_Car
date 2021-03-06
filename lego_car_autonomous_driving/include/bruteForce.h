#include "ros/ros.h"
#include "lego_car_msgs/EnginePower.h"
#include "sensor_msgs/Range.h"
#include <map>

class BruteForceDriver
{
private:
    static bool mvRunning;
    ros::Subscriber mvSubscriber;
    ros::Publisher mvPublisher;
    ros::Rate *mvLoopRate;
    float mvDistance;
    void receiveMessage(const sensor_msgs::Range::ConstPtr& msg);
    static void shutdown(int sig);

public:
    BruteForceDriver(ros::NodeHandle *n);
    void run();
    ~BruteForceDriver();
};


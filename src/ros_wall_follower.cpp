#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#define FLOAT32_ARRAY_SIZE(x) sizeof(x)/sizeof(float)
#define MAX_SCAN_POINTS 1080
#define NUM_REGIONS 5
#define MAX_RANGE 30 // [m]

// using namespace std;
using sensor_msgs::LaserScan;
using nav_msgs::Odometry;
using ackermann_msgs::AckermannDriveStamped;

typedef float float32_t;
typedef double float64_t;

typedef struct laser_read_S
{   
    float angle_increment;
    float min_ranges[5];
    std::vector<float> ranges;
}laser_read_s;

class WallFollower
{
private:
    ros::Timer            _timer;
    ros::Publisher        _drive_pub;
    ros::Subscriber       _laser_sub;
    LaserScan             _laser;
    AckermannDriveStamped _car;
    Odometry              _position;

    void _scanCallback(const sensor_msgs::LaserScan &msg);
    void _timerCallback(const ros::TimerEvent &event);

public:
    WallFollower(int argc, char** argv);
    ~WallFollower();
    void setCarMovement(float32_t steering_angle, float32_t steering_angle_velocity, \
                                    float32_t speed, float32_t acceleration, float32_t jerk);
    void publishMsg(void);
    void getScanRanges(void);
    laser_read_s laser_read;
    // ros::Publisher drive_pub;
};

WallFollower::WallFollower(int argc, char** argv) : _drive_pub(), _laser(), _car(), _position()
{
    std::string node_name = "drive_car";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    _drive_pub = nh.advertise<AckermannDriveStamped>("drive", 10);
    _laser_sub = nh.subscribe("scan", 10, &WallFollower::_scanCallback, this);
    _timer     = nh.createTimer(ros::Duration(1.0), &WallFollower::_timerCallback, this);
}

WallFollower::~WallFollower()
{
}

void WallFollower::_scanCallback(const sensor_msgs::LaserScan &msg)
{
    _laser = msg;
    laser_read.angle_increment = _laser.angle_increment;
    laser_read.ranges = _laser.ranges;
    getScanRanges();
}

void WallFollower::_timerCallback(const ros::TimerEvent &event)
{
    // ROS_INFO("Time elapsed: %f", event.last_real.now().toSec());
}

void WallFollower::getScanRanges(void)
{
    int count = 0;
    for (int i = 1; i < laser_read.ranges.size(); i += (laser_read.ranges.size() / NUM_REGIONS))
    {
        float min_val = laser_read.ranges[i];
        for (int j = i; j < (laser_read.ranges.size() / NUM_REGIONS); j++)
        {
            min_val = (laser_read.ranges[j] < min_val) ? laser_read.ranges[j] : min_val;
        }
        laser_read.min_ranges[count] = min_val;
        count++;
    }
    ROS_INFO("DER: %f, ANG_DER: %f, FRENTE: %f, ANG_IZQ: %f, IZQ: %f",
             laser_read.min_ranges[0], laser_read.min_ranges[1], laser_read.min_ranges[2], laser_read.min_ranges[3], laser_read.min_ranges[4]);
    // ROS_INFO("Vector size: %u", ranges.size());
    // ROS_INFO("Angle Increment: %f", angle_increment);
}

void WallFollower::setCarMovement(float32_t steering_angle, float32_t steering_angle_velocity, \
                                    float32_t speed, float32_t acceleration, float32_t jerk)
{
    _car.drive.steering_angle          = steering_angle;
    _car.drive.steering_angle_velocity = steering_angle_velocity;
    _car.drive.speed                   = speed;
    _car.drive.acceleration            = acceleration;
    _car.drive.jerk                    = jerk;
}

void WallFollower::publishMsg(void)
{
    _drive_pub.publish(_car);
}

int main(int argc, char** argv)
{
    WallFollower robot(argc, argv);
    ros::spin();
    return 0;
}
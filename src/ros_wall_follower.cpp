#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <fstream>
#include <iostream>

#define FLOAT32_ARRAY_SIZE(x) sizeof(x)/sizeof(float)
#define MAX_SCAN_POINTS 1080
#define MAX_RANGE 30 // [m]

#define DER 0
#define FR_DER 1
#define FRONT 2
#define FR_IZQ 3
#define IZQ 4
#define NUM_REGIONS 5

#define CSV_RATE 1 // [s]

// using namespace std;
using sensor_msgs::LaserScan;
using nav_msgs::Odometry;
using ackermann_msgs::AckermannDriveStamped;

typedef float float32_t;
typedef double float64_t;

typedef struct laser_read_S
{   
    float32_t angle_increment;
    float32_t min_ranges[5];
    std::vector<float32_t> ranges;
}laser_read_s;

typedef struct pid_control_S
{
    float32_t dt;
    float32_t setpoint;
    float32_t error[2];
    struct
    {
        float32_t Kp;
        float32_t Ki;
        float32_t Kd;
    } gains;
} pid_control_s;

typedef struct steering_params_S
{
    float32_t output;
    float32_t max_value;
} steering_params_s;

typedef struct ranges_index_S
{
    int begin;
    int end;
} ranges_index_s;

typedef struct odom_coord_S
{
    float32_t x;
    float32_t y;
} odom_coord_s;

class WallFollower
{
private:
    ros::Timer            _timer0;
    ros::Timer            _timer1;
    ros::Publisher        _drive_pub;
    ros::Subscriber       _laser_sub;
    ros::Subscriber       _odom_sub;
    LaserScan             _laser;
    AckermannDriveStamped _car;
    Odometry              _odom;

    void _scanCallback(const sensor_msgs::LaserScan &msg);
    void _odomCallback(const nav_msgs::Odometry &msg);
    void _timer0Callback(const ros::TimerEvent &event);
    void _timer1Callback(const ros::TimerEvent &event);

public:
    WallFollower(int argc, char** argv);
    ~WallFollower();
    void setCarMovement(float32_t steering_angle, float32_t steering_angle_velocity, \
                                    float32_t speed, float32_t acceleration, float32_t jerk);
    void getScanRanges(void);
    void calculateControl(void);
    void takeAction(void);
    void publishAckermannMsg(void);
    void outputCSV(void);

    laser_read_s              laser_read;
    pid_control_s             control;
    steering_params_s         steer;
    float32_t                 car_speed;
    std::vector<odom_coord_s> waypoints;
    std::fstream              fout;
    uint                      csv_count;
    const ranges_index_s      scan_points[NUM_REGIONS] =
    {
        {107, 322}, // 0 - DER
        {323, 420}, // 1 - FR_DER
        {421, 660}, // 2 - FRONT
        {661, 751}, // 1 - FR_DER
        {752, 967}  // 4 - IZQ
    };
    
};

WallFollower::WallFollower(int argc, char** argv) : _drive_pub(), _laser(), _car(), _odom()
{
    std::string node_name = "ros_wall_follower";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    control    =
        {
            dt       : 0.1,
            setpoint : 2.0,
            error    : {0.0, 0.0},
            gains    : {
                1.0,    // Kp
                0.0025, // Ki
                0.005   // Kd
                }
        };
    steer =
        {
            output : 0.0,
            max_value : 1.0
        };
    csv_count     = 0;
    car_speed     = 2.0;
    _drive_pub    = nh.advertise<AckermannDriveStamped>("drive", 10);
    _laser_sub    = nh.subscribe("scan", 10, &WallFollower::_scanCallback, this);
    _odom_sub     = nh.subscribe("odom", 10, &WallFollower::_odomCallback, this);
    _timer0       = nh.createTimer(ros::Duration(control.dt), &WallFollower::_timer0Callback, this);
    _timer1 = nh.createTimer(ros::Duration(CSV_RATE), &WallFollower::_timer1Callback, this);
    fout.open("odom_data.csv", std::ios::out);
}

WallFollower::~WallFollower()
{
    fout.close();
}

void WallFollower::_scanCallback(const sensor_msgs::LaserScan &msg)
{
    _laser = msg;
    laser_read.angle_increment = _laser.angle_increment;
    laser_read.ranges = _laser.ranges;
    getScanRanges();
}

void WallFollower::_timer0Callback(const ros::TimerEvent &event)
{
    // ROS_INFO("Time elapsed: %f", event.last_real.now().toSec());
    takeAction();
}

void WallFollower::_timer1Callback(const ros::TimerEvent &event)
{
    odom_coord_s coord;
    coord.x = _odom.pose.pose.position.x;
    coord.y = _odom.pose.pose.position.y;
    waypoints.push_back(coord);
    outputCSV();
}

void WallFollower::_odomCallback(const nav_msgs::Odometry &msg)
{
    _odom = msg;
}

void WallFollower::getScanRanges(void)
{
    for (int i = 0; i < NUM_REGIONS; i++)
    {
        int j = scan_points[i].begin;
        float32_t min_val = laser_read.ranges[j];
        for (; j <= scan_points[i].end; j++)
        {
            min_val = (laser_read.ranges[j] < min_val) ? laser_read.ranges[j] : min_val;
        }
        laser_read.min_ranges[i] = min_val;   
    }

    // ROS_INFO("DER: %f, ANG_DER: %f, FRENTE: %f, ANG_IZQ: %f, IZQ: %f",
    //          laser_read.min_ranges[DER], laser_read.min_ranges[FR_DER], laser_read.min_ranges[FRONT], \
    //          laser_read.min_ranges[FR_IZQ], laser_read.min_ranges[IZQ]);
}

void WallFollower::calculateControl(void)
{
    // control.setpoint = (laser_read.min_ranges[DER] + laser_read.min_ranges[IZQ]) / 2; // CENTRO: ESTA LINEA Y ABAJO
    control.error[0] = control.setpoint - laser_read.min_ranges[DER]; // DERECHA SOLO ESTA LINEA
    float32_t Up = control.gains.Kp * control.error[0];
    float32_t Ui = control.gains.Ki * (control.error[0] - control.error[1]) / 2;
    float32_t Ud = control.gains.Kd * (1 / control.dt) * (control.error[0] - control.error[1]);
    float32_t U  = Up + Ui + Ud;
    
    steer.output = (((steer.max_value) / (control.gains.Kp * 2)) * U);

    int sign = steer.output >= 0 ? 1 : -1;

    steer.output = abs(steer.output) >= (steer.max_value) ? \
                    (sign * steer.max_value) : (steer.output);

    control.error[1] = control.error[0];
}

void WallFollower::takeAction(void)
{
    calculateControl();
    if (laser_read.min_ranges[FRONT] <= 0.2)
    {
        car_speed = -5.0;
        steer.output = (steer.output >= 0) ? (-1.0 * steer.max_value) : steer.max_value;
    }
    else if ((laser_read.min_ranges[FRONT] > 0.2) && (laser_read.min_ranges[FRONT] <= 0.9))
    {
        int steer_sign = (steer.output >= 0) ? (-1.0 * steer.max_value) : (steer.max_value) ;
        steer.output = (car_speed == -5.0) ? steer_sign : steer.output;
    }
    else if ((laser_read.min_ranges[FRONT] > 0.9) && (laser_read.min_ranges[FRONT] <= 1.4))
    {
        car_speed = 1.0;
    }
    else if ((laser_read.min_ranges[FRONT] > 1.4))
    {
        car_speed = 2.0;
    }
    setCarMovement(steer.output, 0.1, car_speed, 0.0, 0.0);
    publishAckermannMsg();
    ROS_INFO("%f, %f, %f, %f", _car.drive.steering_angle, _car.drive.speed, \
                laser_read.min_ranges[DER], laser_read.min_ranges[FRONT]);
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

void WallFollower::publishAckermannMsg(void)
{
    _drive_pub.publish(_car);
}

void WallFollower::outputCSV(void)
{
    fout << waypoints[csv_count].x << "," << waypoints[csv_count].y << "\n";
    ++csv_count;
}

int main(int argc, char** argv)
{
    WallFollower robot(argc, argv);
    ros::spin();
    return 0;
}
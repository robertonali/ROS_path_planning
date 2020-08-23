#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include <iterator>

#define FLOAT32_ARRAY_SIZE(x) sizeof(x)/sizeof(float)
#define MAX_SCAN_POINTS 1080
#define MAX_RANGE 30    // [m]
#define CSV_RATE 0.1f   // [s] 

#define QX odom.quaternion.x
#define QY odom.quaternion.y
#define QZ odom.quaternion.z
#define QW odom.quaternion.w

// using namespace std;
using sensor_msgs::LaserScan;
using nav_msgs::Odometry;
using ackermann_msgs::AckermannDriveStamped;

typedef float float32_t;
typedef double float64_t;

/*
/   LaserScan Data Structs/Enums
*/
typedef enum regions_E
{
    DER = 0,
    IZQ,
    NUM_REGIONS   
}regions_e;

typedef struct ranges_index_S
{
    int begin;
    int end;
} ranges_index_s;

typedef struct centroid_S
{
    float32_t sum_moment, sum_x, x, normalized;
}centroid_s;

typedef struct laser_read_S
{   
    float32_t angle_increment;
    float32_t min_ranges[5];
    std::vector<float32_t> ranges;
    centroid_s centroid;
}laser_read_s;

/*
/   Odometry Data Structs/Enums
*/

typedef enum euler_E
{
    ROLL = 0,       // X
    PITCH,          // Y
    YAW,            // Z
    EULER_ANGLES
} euler_e;

typedef struct quaternions_S
{
    float64_t x, y, z, w;
} quaternions_s;

typedef struct odom_coord_S
{
    float64_t x, y, magnitude;
} odom_coord_s;

typedef struct odom_data_S
{
    odom_coord_s current_pos;
    odom_coord_s prev_pos;      // Only needed when taking waypoint per distance.
    odom_coord_s current_vel;
    quaternions_s quaternion;
    float64_t euler[EULER_ANGLES];
    std::vector<odom_coord_s> waypoints;
} odom_data_s;

/*
/   PID Control Data Structs/Enums
*/

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
    float32_t output, max_value;
} steering_params_s;

typedef struct car_S
{
    steering_params_s steer;
    float32_t wheelbase, track, speed;
} car_s;

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
    void getScanCentroid(void);
    float32_t calculateControl(float32_t);
    void takeAction(void);
    void publishAckermannMsg(void);
    void outputCSV(void);
    void getWaypoints(void);

    odom_data_s               odom;
    laser_read_s              laser;
    pid_control_s             control;
    car_s                     car;
    float32_t                 car_speed;
    std::fstream              fout;
    unsigned int              csv_count;
    const ranges_index_s      scan_points[NUM_REGIONS] =
    {
        {140, 539}, // 0 - DER
        {540, 940}  // 1 - IZQ
    };
    
};

WallFollower::WallFollower(int argc, char** argv) : _drive_pub(), _laser(), _car(), _odom()
{
    std::string node_name = "dummy_agent";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    memset(&odom, 0, sizeof(odom));
    memset(&laser, 0, sizeof(laser));
    control =
        {
            dt       : 0.01,
            setpoint : 1.5,
            error    : {0.0, 0.0},
            gains    : {
                1.70,   // Kp 1.2
                0.00,   // Ki 0.0025
                0.40    // Kd 0.0005
                }
        };
    car =
        {
            steer     : {
                output    : 0.0,
                max_value : 1.0
            },
            wheelbase : 0.40,
            track     : 0.28,
            speed     : 3.0
        };
    csv_count  = 0;
    _drive_pub = nh.advertise<AckermannDriveStamped>("drive", 1);
    _laser_sub = nh.subscribe("scan", 1, &WallFollower::_scanCallback, this);
    _odom_sub  = nh.subscribe("odom", 1, &WallFollower::_odomCallback, this);
    _timer0    = nh.createTimer(ros::Duration(control.dt), &WallFollower::_timer0Callback, this);
    // _timer1    = nh.createTimer(ros::Duration(CSV_RATE), &WallFollower::_timer1Callback, this);
    fout.open("./src/ros_wall_follower/src/odom_data.csv", std::ios::out);
}

WallFollower::~WallFollower()
{
    fout.close();
}

void WallFollower::_scanCallback(const sensor_msgs::LaserScan &msg)
{
    _laser = msg;
    // laser.angle_increment = _laser.angle_increment;
    laser.angle_increment = 0.25;
    laser.ranges = _laser.ranges;
    // getScanRanges();
    getScanCentroid();
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
    odom.waypoints.push_back(coord);
    outputCSV();
}

void WallFollower::_odomCallback(const nav_msgs::Odometry &msg)
{
    _odom = msg;
    odom.current_pos.x = _odom.pose.pose.position.x;
    odom.current_pos.y = _odom.pose.pose.position.y;
    odom.current_vel.x = _odom.twist.twist.linear.x;
    odom.current_vel.y = _odom.twist.twist.linear.y;
    odom.current_vel.magnitude = hypot(odom.current_vel.x, odom.current_vel.y);

    // Method 1 
    // struct {                        // Not valid due to unamed struct does not corresponds to quaternion_s when assigning
    //     float64_t x, y, z, w;
    // } orientation = {
    quaternions_s orientation = {
        x : _odom.pose.pose.orientation.x,
        y : _odom.pose.pose.orientation.y,
        z : _odom.pose.pose.orientation.z,
        w : _odom.pose.pose.orientation.w,
    };
    odom.quaternion = orientation;

    // Method 2
    // std::tie(odom.quaternion.x, odom.quaternion.y, odom.quaternion.z, odom.quaternion.w) \
    //         = std::make_tuple(_odom.pose.pose.orientation.x, _odom.pose.pose.orientation.y, \
    //                             _odom.pose.pose.orientation.z, _odom.pose.pose.orientation.w);

    float64_t euler_from_quaternion[] = {
        atan2((2 * (QW * QX + QY * QZ)), (1 - 2 * (QX * QX + QY * QY))),
        asin(2 * ((QW * QY) - (QX * QZ))),
        atan2((2 * (QW * QZ + QX * QY)), (1 - 2 * (QY * QY + QZ * QZ)))
    };
    // odom.euler = euler_angles; // Invalid array assignment
    memcpy(odom.euler, euler_from_quaternion, sizeof(odom.euler));
}

void WallFollower::getScanRanges(void)
{
    for (int i = 0; i < NUM_REGIONS; i++)
    {
        int j = scan_points[i].begin;
        float32_t min_val = laser.ranges[j];
        for (; j <= scan_points[i].end; j++)
        {
            min_val = (laser.ranges[j] < min_val) ? laser.ranges[j] : min_val;
        }
        laser.min_ranges[i] = min_val;   
    }

    // ROS_INFO("DER: %f, ANG_DER: %f, FRENTE: %f, ANG_IZQ: %f, IZQ: %f",
    //          laser.min_ranges[DER], laser.min_ranges[FR_DER], laser.min_ranges[FRONT], \
    //          laser.min_ranges[FR_IZQ], laser.min_ranges[IZQ]);
}

void WallFollower::getScanCentroid(void)
{
    laser.centroid.sum_moment = 0.0;
    laser.centroid.sum_x      = 0.0;
    auto start_iter     = next(laser.ranges.begin(), scan_points[DER].begin);
    auto end_iter       = next(laser.ranges.begin(), scan_points[IZQ].end);
    unsigned int index  = scan_points[DER].begin;
    // [&] "Captures" external variables as reference into the lambda functions. Can pass [&index] alone, or any other variable
    std::for_each(start_iter, end_iter, [&] (const float32_t value)
    {
        laser.centroid.sum_moment += (index * value);
        laser.centroid.sum_x += value;
        ++index; 
    } );
    // for (int i = 0; i < NUM_REGIONS; i++)
    // {
    //     for (int j = scan_points[i].begin; j <= scan_points[i].end; j++)
    //     {
    //         laser.centroid.sum_moment += (j * laser.ranges[j]);
    //         laser.centroid.sum_x += laser.ranges[j]; // x (in deg) * f(x) * dx
    //     } 
    // }
    laser.centroid.x = laser.centroid.sum_moment / laser.centroid.sum_x;
    // Normalize centroid
    laser.centroid.normalized = ((laser.centroid.x / 400) - 1.35);
}

float32_t WallFollower::calculateControl(float32_t centroid)
{
    // control.error[0] = control.setpoint - laser.min_ranges[DER];
    control.setpoint = 0; // scan_points[1].begin;
    control.error[0] = centroid - control.setpoint;
    float32_t Up = control.gains.Kp * control.error[0];
    float32_t Ui = control.gains.Ki * control.dt * (control.error[0] - control.error[1]) / 2;
    float32_t Ud = control.gains.Kd * (1 / control.dt) * (control.error[0] - control.error[1]);
    float32_t U  = Up + Ui + Ud;

    control.error[1] = control.error[0];
    
    // steer.output = (((steer.max_value) / (control.gains.Kp * 270)) * U);

    // int sign = steer.output >= 0 ? 1 : -1;

    // steer.output = abs(steer.output) >= (steer.max_value) ? \
    //                 (sign * steer.max_value) : (steer.output);

    // return std::min(std::max(-1.0, U), 1.0);
    return fmin(fmax(-1.0, U), 1.0);
}

void WallFollower::takeAction(void)
{
    car.steer.output = calculateControl(laser.centroid.normalized);
    setCarMovement(car.steer.output, 0.0, car.speed, 0.0, 0.0);
    publishAckermannMsg();
    getWaypoints();
    // ROS_INFO("%f, %f, %f, %f", _car.drive.steering_angle, _car.drive.speed, \
    //             laser.min_ranges[DER], laser.min_ranges[FRONT]);

    ROS_INFO("Centroid: %f, Setpoint: %d, Steer: %f, Speed: %f, Error: %f", laser.centroid.normalized, scan_points[1].begin,\
                _car.drive.steering_angle, _car.drive.speed, control.error[0]);
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
    fout << odom.waypoints[csv_count].x << "," << odom.waypoints[csv_count].y << "\n";
    ++csv_count;
}

void WallFollower::getWaypoints(void)
{
    if (hypot((odom.current_pos.x - odom.prev_pos.x), (odom.current_pos.y - odom.prev_pos.y)) \
                >= (car.wheelbase * 0.8))
    {
        odom_coord_s coord;
        odom.prev_pos.x = odom.current_pos.x;
        odom.prev_pos.y = odom.current_pos.y;
        coord.x = odom.current_pos.x + ((car.wheelbase / 2) * cos(odom.euler[YAW]));
        coord.y = odom.current_pos.y +((car.wheelbase / 2) * sin(odom.euler[YAW]));
        odom.waypoints.push_back(coord);
        outputCSV();
    }

}

int main(int argc, char** argv)
{
    WallFollower robot(argc, argv);
    ros::spin();
    return 0;
}
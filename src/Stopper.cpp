#include "my_obstacle_avoidance/Stopper.h"
#include "geometry_msgs/Twist.h"

Stopper::Stopper()
{
  keep_moving_ = true;

  float s;

  if (node_.getParam("min_distance", par_min_distance_)) {
    ROS_INFO("Got parameter min_distance = %f", par_min_distance_);
  } else {
    par_min_distance_ = DEFAULT_MIN_DISTANCE;
    ROS_INFO("Using min_distance default = %f", par_min_distance_);  
  }

  // Advertise a new publisher for the robot's velocity command topic
  twistPub_ = node_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 10);

  // Subscribe to the simulated robot's laser scan topic
  scanSub_ = node_.subscribe("scan", 1, &Stopper::scanCallback, this);
  
}

Stopper::~Stopper()
{
}


// Process the incoming laser scan message
void Stopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    ROS_INFO("Range[0] is %f", scan->ranges[0]);
    if (scan->ranges[0] < par_min_distance_) {
        ROS_INFO("Stop!");
        keep_moving_ = false;
    }
}


// Send a velocity command
void Stopper::moveForward() {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = 0.2;
    twistPub_.publish(msg);
}

void Stopper::startMoving() 
{
  ros::Rate rate(10);
  ROS_INFO("Start moving");

  // Keep spinning loop until user presses Ctrl+C or the robot got too close to an obstacle
  while (ros::ok() && keep_moving_) {
    moveForward();
    ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
    rate.sleep();
  }
}
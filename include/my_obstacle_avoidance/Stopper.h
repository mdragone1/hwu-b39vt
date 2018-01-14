#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define DEFAULT_MIN_DISTANCE 0.5

class Stopper
{
public:
	Stopper();
	virtual ~Stopper();
	void startMoving(); // start moving the robot at a fixed velocity until an obstacle is on the way

private:
	ros::NodeHandle node_; // Handle to the ROS node
	ros::Publisher twistPub_; // Publisher to the robot's velocity command
	ros::Subscriber scanSub_; // Subscriber to the robot's laser scan topic
	
	double par_min_distance_;
	bool keep_moving_; // indicates whether the robot should continue moving
	void moveForward(); // tell the robot to move forward (by publishing the corresponding twist message to control robot's velocity)
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan); // callback for receiving scan updates
};
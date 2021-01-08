//
// Created by lihao on 21-1-6.
//

#ifndef __CAR_HPP__
#define __CAR_HPP__


#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>


using namespace std;


//------------------------------- Define -------------------------------//
#define C_PI (double) 3.141592653589793
#define DEG2RAD(DEG) ((DEG)*((C_PI)/(180.0)))
#define RAD2DEG(RAD) ((RAD)*((180.0)/(C_PI)))

namespace robot{

class Car{

public:
    bool setup(ros::NodeHandle nh, ros::NodeHandle nh_priv);
    void spin();

private:
    void VelCallback(const geometry_msgs::Twist& msg);
    void update();
    void PublishOdometry();
    void PublishPath();
    void PublishTF();
    void PublishStaticTF();
    void PublishPolygon();

private:
    ros::Subscriber vel_sub;
    ros::Publisher odom_pub;
    ros::Publisher path_pub;
    ros::Publisher polygon_pub;

    nav_msgs::Path path;
    geometry_msgs::Pose2D init_pose;
    geometry_msgs::Pose2D pose;
    geometry_msgs::Twist velocity;
    geometry_msgs::Polygon polygon;

    /* parameter */
    string map_frame;
    string odom_frame;
    string base_frame;
    double duration;
};

}


#endif
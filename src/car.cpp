#include "car.hpp"


namespace robot
{
    
bool Car::setup(ros::NodeHandle nh, ros::NodeHandle nh_priv)
{
    nav_finish_flag = false;
    init_pose = geometry_msgs::Pose2D();
    pose = geometry_msgs::Pose2D();
    velocity = geometry_msgs::Twist();

    // Parameter
    vector<double> _polygon;
    nh_priv.param<double>("duration", duration, 0.1);
    nh_priv.param<string>("map_frame", map_frame, "map");
    nh_priv.param<string>("odom_frame", odom_frame, "odom");
    nh_priv.param<string>("base_frame", base_frame, "base_link");
    nh_priv.param<double>("x", init_pose.x, 0);      // Initial pose of car
    nh_priv.param<double>("y", init_pose.y, 0);
    nh_priv.param<double>("theta", pose.theta, 0);
    nh_priv.param<double>("vx", velocity.linear.x, 0); // Initial velocity of car
    nh_priv.param<double>("vy", velocity.linear.y, 0);
    nh_priv.param<double>("vth", velocity.angular.z, 0);
    nh_priv.getParam("max_vel", max_vel);
    nh_priv.getParam("polygon", _polygon);

    pose.theta = DEG2RAD(pose.theta);
    for(int i=0;i<_polygon.size() / 2;i++)
    {
        geometry_msgs::Point32 point;
        point.x = _polygon[2 * i];
        point.y = _polygon[2 * i + 1];
        polygon.points.push_back(point);
    }

    // Subscribe velocity topic
    vel_sub = nh.subscribe("cmd_vel", 10, &Car::VelCallback, this);
    goalreached_sub = nh.subscribe("isgoalreached", 10, &Car::GoalReachedCallback, this);

    // Advertise odometry topic
    odom_pub = nh_priv.advertise<nav_msgs::Odometry>("odom", 10);
    path_pub = nh_priv.advertise<nav_msgs::Path>("path", 10);
    polygon_pub = nh_priv.advertise<geometry_msgs::PolygonStamped>("polygon", 10);

    path = nav_msgs::Path();
    path.header.stamp = ros::Time::now();
    path.header.frame_id = odom_frame;

    // Publish static TF
    PublishStaticTF();

    return true;
}

void Car::spin()
{
    while(ros::ok())
    {
        update();
        PublishOdometry();
        PublishPath();
        PublishTF();
        PublishPolygon();

        // Sleep and wait for callback
        ros::Duration(duration).sleep();
        ros::spinOnce();
    }    
}

void Car::VelCallback(const geometry_msgs::Twist& msg)
{
    if(nav_finish_flag)
    {
        velocity = geometry_msgs::Twist();
    }
    else
    {
        velocity.linear.x = abs(msg.linear.x) > max_vel[0] ? (msg.linear.x > 0 ? max_vel[0] : -max_vel[0]) : msg.linear.x;
        velocity.linear.y = abs(msg.linear.y) > max_vel[1] ? (msg.linear.y > 0 ? max_vel[1] : -max_vel[1]) : msg.linear.y;
        velocity.angular.z = abs(msg.angular.x) > max_vel[2] ? (msg.angular.x > 0 ? max_vel[2] : -max_vel[2]) : msg.angular.x;

        // ROS_INFO("velocity: %f, %f, %f", velocity.linear.x, velocity.linear.y, velocity.linear.z);
    }
}

void Car::GoalReachedCallback(const std_msgs::Bool& msg)
{
    if(!nav_finish_flag && msg.data)
        ROS_INFO("Finishing navigation");
    nav_finish_flag = msg.data;
}

void Car::update()
{
    // Calculate the pose of the car
    double vx = velocity.linear.x;
    double vy = velocity.linear.y;
    double vth = velocity.angular.z;

    double dt = duration;

    double dx = (vx * cos(pose.theta) - vy * sin(pose.theta)) * dt;
    double dy = (vx * sin(pose.theta) + vy * cos(pose.theta)) * dt;
    double dth = vth * dt;

    pose.x += dx;
    pose.y += dy;
    pose.theta += dth;
    pose.theta = fmod(pose.theta, 2 * C_PI);    
}

void Car::PublishOdometry()
{
    // Publish odometry topic
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = odom_frame;

    odom.pose.pose.position.x = pose.x;
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose.theta);

    odom.child_frame_id = base_frame;
    odom.twist.twist.linear.x = velocity.linear.x;
    odom.twist.twist.linear.y = velocity.linear.y;
    odom.twist.twist.linear.z = 0.0;    
    odom.twist.twist.angular.z = velocity.angular.z;

    odom_pub.publish(odom);
}

void Car::PublishPath()
{
    // Publish path topic
    geometry_msgs::PoseStamped pose_stamped = geometry_msgs::PoseStamped();
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = odom_frame;
    pose_stamped.pose.position.x = pose.x;
    pose_stamped.pose.position.y = pose.y;
    pose_stamped.pose.position.z = 0.0;      
    path.poses.push_back(pose_stamped);
    path_pub.publish(path);
}

void Car::PublishTF()
{
    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = odom_frame;
    transformStamped.child_frame_id = base_frame;
    transformStamped.transform.translation.x = pose.x;
    transformStamped.transform.translation.y = pose.y;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation = tf::createQuaternionMsgFromYaw(pose.theta);
    broadcaster.sendTransform(transformStamped);
}

void Car::PublishStaticTF()
{
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = map_frame;
    static_transformStamped.child_frame_id = odom_frame;
    static_transformStamped.transform.translation.x = init_pose.x;
    static_transformStamped.transform.translation.y = init_pose.y;
    static_transformStamped.transform.translation.z = 0.0;
    static_transformStamped.transform.rotation = tf::createQuaternionMsgFromYaw(init_pose.theta);
    static_broadcaster.sendTransform(static_transformStamped);
}

void Car::PublishPolygon()
{
    geometry_msgs::PolygonStamped polygonStamped;
    polygonStamped.header.stamp = ros::Time::now();
    polygonStamped.header.frame_id = base_frame;
    polygonStamped.polygon = polygon;
    polygon_pub.publish(polygonStamped);
}

} 
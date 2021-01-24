#! /usr/bin/env python
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_geometry_msgs import PoseStamped
import tf2_ros
import tf
from pid import Pid, PidParam


deg2rad = lambda x: x * np.pi / 180.0
rad2deg = lambda x: x * 180.0 / np.pi


# Global variable
global poseStamped
poseStamped = None


def OdometryCallback(msg):
    global poseStamped
    poseStamped = PoseStamped()
    poseStamped.header = msg.header
    poseStamped.pose = msg.pose.pose    


if __name__ == '__main__':
    # Initial node
    rospy.init_node('pidtrack', anonymous=True)
    rospy.loginfo('Start the pidtrack node')

    # Parameter
    pidParam = PidParam()

    base_frame = rospy.get_param('~base_frame', 'base_link')     
    pidParam.KP = rospy.get_param('~KP') 
    pidParam.KI = rospy.get_param('~KI')
    pidParam.KD = rospy.get_param('~KD')
    pidParam.duration = rospy.get_param('~duration', 0.1)
    pidParam.windup = rospy.get_param('~windup', 100.0)
    pidTarget = rospy.get_param('~pidTarget')

    pidTarget[3] = deg2rad(pidTarget[3])
    pidTarget = np.asarray(pidTarget)
    duration = pidParam.duration

    # Initial Pid
    pid = Pid(pidParam)

    # Subscribe topics
    rospy.Subscriber('odom', Odometry, OdometryCallback, queue_size=10)

    # Advertise cmd_vel topic
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # TF
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # Loop
    while not rospy.is_shutdown():
        # Transform
        if poseStamped:
            try:
                poseStamped = tfBuffer.transform(poseStamped, base_frame, timeout=rospy.Duration(duration))
            except:
                # rospy.logerr('Transform failed')
                continue
            yaw = tf.transformations.euler_from_quaternion([poseStamped.pose.orientation.x, poseStamped.pose.orientation.y, 
                                                    poseStamped.pose.orientation.z, poseStamped.pose.orientation.w])[2]
            pidInput = np.array([poseStamped.pose.position.x, poseStamped.pose.position.y, poseStamped.pose.position.z, yaw])
            
            # PID control
            pidOutput = pid.CalcPid(pidInput, pidTarget)

            cmd_vel = Twist()
            cmd_vel.linear.x = -pidOutput[0]
            cmd_vel.linear.y = -pidOutput[1]
            cmd_vel.linear.z = -pidOutput[2]
            cmd_vel.angular.z = -pidOutput[3]
            cmd_vel_pub.publish(cmd_vel)

        # Sleep and wait for callback
        rospy.sleep(duration)

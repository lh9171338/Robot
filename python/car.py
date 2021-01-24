import numpy as np
import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from std_msgs.msg import Bool
import tf2_ros
import tf

deg2rad = lambda x: x * np.pi / 180.0
rad2deg = lambda x: x * 180.0 / np.pi

class Car:
    def __init__(self):
        self.nav_finish_flag = False
        self.init_pose = Pose2D()
        self.pose = Pose2D()
        self.velocity = Twist()
        self.polygon = Polygon()

    def setup(self):

        # Parameter
        self.duration = rospy.get_param('~duration', 0.1)
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.init_pose.x = rospy.get_param('~x', 0.0)  # Initial pose of drone
        self.init_pose.y = rospy.get_param('~y', 0.0)
        self.pose.theta = rospy.get_param('~theta', 0.0) 
        self.velocity.linear.x = rospy.get_param('~vx', 0.0)   # Initial velocity of drone
        self.velocity.linear.y = rospy.get_param('~vy', 0.0)
        self.velocity.angular.z = rospy.get_param('~vyaw', 0.0)
        self.max_vel = rospy.get_param('~max_vel', [1.0, 1.0, 1.0])
        polygon = rospy.get_param('~polygon')
        
        self.pose.theta = deg2rad(self.pose.theta)
        for i in range(len(polygon) // 2):
            point = Point32()
            point.x = polygon[2 * i]
            point.y = polygon[2 * i + 1]   
            self.polygon.points.append(point)

        # Subscribe velocity topic
        rospy.Subscriber('cmd_vel', Twist, self.VelCallback, queue_size=10)
        rospy.Subscriber('isgoalreached', Bool, self.GoalReachedCallback, queue_size=10)

        # Advertise odometry topic
        self.odom_pub = rospy.Publisher('~odom', Odometry, queue_size=10)
        self.path_pub = rospy.Publisher('~path', Path, queue_size=10)
        self.polygon_pub = rospy.Publisher('~polygon', PolygonStamped, queue_size=10)

        self.path = Path()
        self.path.header.stamp = rospy.Time().now()
        self.path.header.frame_id = self.odom_frame

        # Publish static TF
        self.PublishStaticTF()

        return True
    
    def spin(self):
        while not rospy.is_shutdown():
            self.update()
            self.PublishOdometry()
            self.PublishPath()
            self.PublishTF()
            self.PublishPolygon()
            
            # Sleep and wait for callback
            rospy.sleep(self.duration)

    def VelCallback(self, msg):
        if self.nav_finish_flag:
            self.velocity = Twist()
        else:
            self.velocity.linear.x = float(np.clip(msg.linear.x, -self.max_vel[0], self.max_vel[0]))
            self.velocity.linear.y = float(np.clip(msg.linear.y, -self.max_vel[1], self.max_vel[1]))
            self.velocity.angular.z = float(np.clip(msg.angular.z, -self.max_vel[2], self.max_vel[2]))

            # rospy.loginfo('velocity: %f, %f, %f', self.velocity.linear.x, self.velocity.linear.y, self.velocity.angular.z)

    def GoalReachedCallback(self, msg):
        if not self.nav_finish_flag and msg.data:
            rospy.loginfo('Finishing navigation')
        self.nav_finish_flag = msg.data

    def update(self):
        # Calculate the pose of the drone
        vx = self.velocity.linear.x
        vy = self.velocity.linear.y
        vth = self.velocity.angular.z
        theta = self.pose.theta
        dt = self.duration

        dx = (vx * np.cos(theta) - vy * np.sin(theta)) * dt
        dy = (vx * np.sin(theta) + vy * np.cos(theta)) * dt
        dth = vth * dt

        self.pose.x += dx
        self.pose.y += dy
        self.pose.theta = (theta + dth) % (2 * np.pi)

    def PublishOdometry(self):
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.pose.theta)

        # Publish odometry topic
        odom = Odometry()
        odom.header.stamp = rospy.Time().now()
        odom.header.frame_id = self.odom_frame

        odom.pose.pose.position.x = self.pose.x
        odom.pose.pose.position.y = self.pose.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.twist.twist = self.velocity
        self.odom_pub.publish(odom)

    def PublishPath(self):
        # Publish path topic
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time().now()
        pose_stamped.header.frame_id = self.odom_frame
        pose_stamped.pose.position.x = self.pose.x
        pose_stamped.pose.position.y = self.pose.y
        pose_stamped.pose.position.z = 0.0
        self.path.poses.append(pose_stamped)
        self.path_pub.publish(self.path)

    def PublishTF(self):
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.pose.theta)

        # Publish tf
        broadcaster = tf2_ros.TransformBroadcaster()
        transformStamped = TransformStamped()
        transformStamped.header.stamp = rospy.Time().now()
        transformStamped.header.frame_id = self.odom_frame
        transformStamped.child_frame_id = self.base_frame
        transformStamped.transform.translation.x = self.pose.x
        transformStamped.transform.translation.y = self.pose.y
        transformStamped.transform.translation.z = 0.0
        transformStamped.transform.rotation.x = quat[0]
        transformStamped.transform.rotation.y = quat[1]
        transformStamped.transform.rotation.z = quat[2]
        transformStamped.transform.rotation.w = quat[3]
        broadcaster.sendTransform(transformStamped)

    def PublishStaticTF(self):
        static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = rospy.Time().now()
        static_transformStamped.header.frame_id = self.map_frame
        static_transformStamped.child_frame_id = self.odom_frame
        static_transformStamped.transform.translation.x = self.init_pose.x
        static_transformStamped.transform.translation.y = self.init_pose.y
        static_transformStamped.transform.translation.z = 0.0
        static_transformStamped.transform.rotation.x = 0
        static_transformStamped.transform.rotation.y = 0
        static_transformStamped.transform.rotation.z = 0
        static_transformStamped.transform.rotation.w = 1
        static_broadcaster.sendTransform(static_transformStamped)

    def PublishPolygon(self):
        polygonStamped = PolygonStamped()
        polygonStamped.header.stamp = rospy.Time().now()
        polygonStamped.header.frame_id = self.base_frame
        polygonStamped.polygon = self.polygon
        self.polygon_pub.publish(polygonStamped)
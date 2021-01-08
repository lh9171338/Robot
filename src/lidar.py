import rospy
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from sensor_msgs.msg import PointField
from nav_msgs.msg import OccupancyGrid
import tf2_ros
import tf
from tf2_geometry_msgs import PoseStamped
from tf2_sensor_msgs import PointCloud2
import numpy as np
import cv2


deg2rad = lambda x: x * np.pi / 180.0
rad2deg = lambda x: x * 180.0 / np.pi


class Lidar:
    def __init__(self):
        self.odom = None
        self.map = None
        self.pointclouds = None

    def setup(self):
        # Parameter
        self.map_frame = rospy.get_param('~map_frame', 'map') 
        self.odom_frame = rospy.get_param('~odom_frame', 'odom') 
        self.base_frame = rospy.get_param('~base_frame', 'base_link') 
        self.publish_frame = rospy.get_param('~publish_frame', 'base_link') 
        self.rate = rospy.get_param('~rate', 20) 
        self.occ_thresh = rospy.get_param('~occ_thresh', 50) 
        self.min_angle = rospy.get_param('~min_angle', 0.0)
        self.max_angle = rospy.get_param('~max_angle', 360.0) 
        self.step_angle = rospy.get_param('~step_angle', 1.0)
        self.min_range = rospy.get_param('~min_range', 0.0) 
        self.max_range = rospy.get_param('~max_range', 10.0) 
        self.min_height = rospy.get_param('~min_height', 0.0) 
        self.max_height = rospy.get_param('~max_height', 4.0)  
        self.step_height = rospy.get_param('~step_height', 0.2)                 
        self.min_angle = deg2rad(self.min_angle)
        self.max_angle = deg2rad(self.max_angle)
        self.step_angle = deg2rad(self.step_angle)

        if self.publish_frame not in [self.map_frame, self.odom_frame, self.base_frame]:
            rospy.logerr('publish_frame must be in [map_frame, odom_frame, base_frame]')
            return False

        # Subscribe odometry topic
        rospy.Subscriber('odom', Odometry, self.OdometryCallback, queue_size=10)

        # Advertise odometry topic
        self.pointcloud2_pub = rospy.Publisher('~pointcloud', PointCloud2, queue_size=10)

        # Process map
        if not self.ProcessMap():
            return False

        # TF
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        return True

    def spin(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.scan()
            self.PublishPointCloud2()
            
            # Sleep and wait for callback
            rate.sleep()
    
    def OdometryCallback(self, msg):
        self.odom = msg
    
    def scan(self):
        self.pointclouds = None
        if self.odom is None:
            return

        # Transform: odom frame -> map frame
        poseStamped = PoseStamped()
        poseStamped.header = self.odom.header
        poseStamped.pose = self.odom.pose.pose
        try:
            poseStamped = self.tfBuffer.transform(poseStamped, self.map_frame, timeout=rospy.Duration(1.0 / self.rate))
        except:
            # rospy.logerr('Transform failed')
            return
        
        # Scan
        pointclouds = []
        yaw = tf.transformations.euler_from_quaternion([poseStamped.pose.orientation.x, poseStamped.pose.orientation.y, 
                                                    poseStamped.pose.orientation.z, poseStamped.pose.orientation.w])[2]
        T = np.array([poseStamped.pose.position.x, poseStamped.pose.position.y])
        height = poseStamped.pose.position.z

        start_angle = yaw + self.min_angle
        end_angle = yaw + self.max_angle
        step_angle = self.step_angle

        start_height = height + self.min_height
        end_height = height + self.max_height
        step_height = self.step_height

        start_range = self.min_range
        end_range = self.max_range
        step_range = self.resolution

        xs = np.arange(start_range, end_range, step_range)
        ys = 0.0 * xs
        ray_points = np.concatenate((xs[:, None], ys[:, None]), axis=-1)

        for angle in np.arange(start_angle, end_angle, step_angle):
            R = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
            transformed_ray_points = (np.matmul(ray_points, R.T) + T) / self.resolution
            transformed_ray_points = np.int32(np.round(transformed_ray_points))
            mask = np.logical_and(np.logical_and(transformed_ray_points[:, 0] >= 0, transformed_ray_points[:, 0] < self.width), 
                                np.logical_and(transformed_ray_points[:, 1] >= 0, transformed_ray_points[:, 1] < self.height))
            transformed_ray_points = transformed_ray_points[mask]
            pointcloud = [np.inf, np.inf]
            if len(transformed_ray_points):
                pixel_values = self.map[transformed_ray_points[:, 1], transformed_ray_points[:, 0]]
                indices = np.argwhere(pixel_values > 0)
                if len(indices) > 0:
                    index = indices[0, 0]
                    pointcloud = [transformed_ray_points[index, 0], transformed_ray_points[index, 1]]

            for height in np.arange(start_height, end_height, step_height):
                pointclouds.append(pointcloud + [height])
            
        pointclouds = np.asarray(pointclouds)
        pointclouds[:, :2] = pointclouds[:, :2] * self.resolution
        self.pointclouds = pointclouds

    def PublishPointCloud2(self):
        if self.pointclouds is None:
            return

        pointCloud2 = PointCloud2()
        pointCloud2.header.stamp = rospy.Time().now()
        pointCloud2.header.frame_id = self.map_frame
        pointCloud2.height = 1
        pointCloud2.width = len(self.pointclouds)
        pointCloud2.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]
        pointCloud2.is_bigendian = False
        pointCloud2.point_step = 12
        pointCloud2.row_step = pointCloud2.point_step * pointCloud2.width
        pointCloud2.is_dense = int(np.isfinite(self.pointclouds).all())
        pointCloud2.data = np.float32(self.pointclouds).tostring()

        try:
            pointCloud2 = self.tfBuffer.transform(pointCloud2, self.publish_frame, timeout=rospy.Duration(1.0 / self.rate))
        except:
            # rospy.logerr('Transform failed')
            return

        self.pointcloud2_pub.publish(pointCloud2)

    def ProcessMap(self):
        # Get map
        rospy.wait_for_service('static_map')
        occupancyGrid = OccupancyGrid()
        try:
            getmap = rospy.ServiceProxy('static_map', GetMap)
            occupancyGrid = getmap().map
        except:
            rospy.logerr('Service call failed')
            return False
        self.resolution = occupancyGrid.info.resolution
        self.height = occupancyGrid.info.height
        self.width = occupancyGrid.info.width
        self.map = np.asarray(occupancyGrid.data).reshape((self.height, self.width))
        self.map = self.map >= self.occ_thresh
        return True

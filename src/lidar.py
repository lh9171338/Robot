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
        self.scan_buffer = None
        self.obstacle_pointclouds = None
        self.map_pointclouds = None

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
        rospy.Subscriber('self/odom', Odometry, self.OdometryCallback, queue_size=10)
        rospy.Subscriber('obstacle/odom', Odometry, self.ObstacleCallback, queue_size=10)

        # Advertise odometry topic
        self.pointcloud2_pub = rospy.Publisher('~pointcloud', PointCloud2, queue_size=10)

        # Process map
        if not self.ProcessMap():
            return False
        self.GenerateScanBuffer()

        # TF
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        return True

    def spin(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.ScanMap()
            self.PublishPointCloud2()
            
            # Sleep and wait for callback
            rate.sleep()
    
    def OdometryCallback(self, msg):
        self.odom = msg
    
    def ObstacleCallback(self, msg):
        if self.odom is None:
            return

        # Transform
        poseStamped = PoseStamped()
        poseStamped.header = msg.header
        poseStamped.pose = msg.pose.pose
        try:
            poseStamped = self.tfBuffer.transform(poseStamped, self.odom_frame, timeout=rospy.Duration(1.0 / self.rate))
        except:
            # rospy.logerr('Transform failed')
            return

        x, y = poseStamped.pose.position.x, poseStamped.pose.position.y
        dx, dy = x - self.odom.pose.pose.position.x, y - self.odom.pose.pose.position.y
        dist = np.sqrt(dx ** 2 + dy ** 2)
        if dist < self.min_range or dist > self.max_range:
            self.obstacle_pointclouds = None
            return
        pointclouds_2d = np.array([[x, y]])

        # 2D -> 3D
        start_height = self.min_height
        end_height = self.max_height
        step_height = self.step_height

        pointclouds = None
        for height in np.arange(start_height, end_height, step_height):
            pointclouds_3d = np.concatenate((pointclouds_2d, height * np.ones((len(pointclouds_2d), 1))), axis=1)
            if pointclouds is None:
                pointclouds = pointclouds_3d
            else:
                pointclouds = np.vstack((pointclouds, pointclouds_3d))

        self.obstacle_pointclouds = pointclouds

    def ScanMap(self):
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
        yaw = tf.transformations.euler_from_quaternion([poseStamped.pose.orientation.x, poseStamped.pose.orientation.y, 
                                                    poseStamped.pose.orientation.z, poseStamped.pose.orientation.w])[2]
        R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])      
        T = np.array([poseStamped.pose.position.x, poseStamped.pose.position.y])
        
        scan_buffer = (np.matmul(self.scan_buffer, R) + T) / self.resolution
        scan_buffer = np.int32(np.round(scan_buffer))
        scan_buffer[:, :, 0] = np.clip(scan_buffer[:, :, 0], 0, self.width - 1)
        scan_buffer[:, :, 1] = np.clip(scan_buffer[:, :, 1], 0, self.height - 1)
        pixel_values = self.map[scan_buffer[:, :, 1], scan_buffer[:, :, 0]]
        mask = pixel_values > 0
        indices = np.where(mask.any(axis=1), mask.argmax(axis=1), -1)

        pointclouds = []
        for index, scan_points in zip(indices, scan_buffer):
            if index == -1:
                pointcloud = [np.inf, np.inf]
            else:
                pointcloud = [scan_points[index, 0], scan_points[index, 1]]
            pointclouds.append(pointcloud)
        pointclouds_2d = np.asarray(pointclouds) * self.resolution

        # 2D -> 3D
        start_height = self.min_height
        end_height = self.max_height
        step_height = self.step_height

        pointclouds = None
        for height in np.arange(start_height, end_height, step_height):
            pointclouds_3d = np.concatenate((pointclouds_2d, height * np.ones((len(pointclouds_2d), 1))), axis=1)
            if pointclouds is None:
                pointclouds = pointclouds_3d
            else:
                pointclouds = np.vstack((pointclouds, pointclouds_3d))

        self.map_pointclouds = pointclouds

    def GenerateScanBuffer(self):
        start_angle = self.min_angle
        end_angle = self.max_angle
        step_angle = self.step_angle 

        start_range = self.min_range
        end_range = self.max_range
        step_range = self.resolution
        xs = np.arange(start_range, end_range, step_range)
        ys = 0.0 * xs
        scan_points = np.concatenate((xs[:, None], ys[:, None]), axis=-1)

        scan_buffer = []
        for angle in np.arange(start_angle, end_angle, step_angle):
            R = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
            rotated_scan_points = np.matmul(scan_points, R.T)
            scan_buffer.append(rotated_scan_points)
        self.scan_buffer = np.asarray(scan_buffer)

    def TransformPointClouds(self, pointclouds, source_frame, target_frame, timeout):
        if target_frame != source_frame:
            try:
                transformStamped = self.tfBuffer.lookup_transform(target_frame, source_frame, 
                    time=rospy.Time().now(), timeout=timeout)
            except:
                # rospy.logerr('Transform failed')
                return
            R = tf.transformations.quaternion_matrix([transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, 
                                                transformStamped.transform.rotation.z, transformStamped.transform.rotation.w])[:3, :3]
            T = np.array([transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z])
            pointclouds = np.matmul(pointclouds, R.T) + T
        return pointclouds

    def GeneratePointCloud2(self, pointclouds, frame_id):
        pointCloud2 = PointCloud2()
        pointCloud2.header.stamp = rospy.Time().now()
        pointCloud2.header.frame_id = frame_id
        pointCloud2.height = 1
        pointCloud2.width = len(pointclouds)
        pointCloud2.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]
        pointCloud2.is_bigendian = False
        pointCloud2.point_step = 12
        pointCloud2.row_step = pointCloud2.point_step * pointCloud2.width
        pointCloud2.is_dense = int(np.isfinite(pointclouds).all())
        pointCloud2.data = np.float32(pointclouds).tostring()
        return pointCloud2

    def PublishPointCloud2(self):
         # Map pointclouds
        map_pointclouds = self.map_pointclouds
        if map_pointclouds is not None:
            map_pointclouds = self.TransformPointClouds(map_pointclouds, self.map_frame, self.publish_frame, 
                                rospy.Duration(1.0 / self.rate))

        # Obstacle pointclouds
        obstacle_pointclouds = self.obstacle_pointclouds
        if obstacle_pointclouds is not None:
            obstacle_pointclouds = self.TransformPointClouds(obstacle_pointclouds, self.odom_frame, self.publish_frame, 
                                rospy.Duration(1.0 / self.rate))

        # Publish PointCloud2 topic
        pointclouds = None
        if map_pointclouds is not None and obstacle_pointclouds is None:
            pointclouds = map_pointclouds
        elif map_pointclouds is None and obstacle_pointclouds is not None:
            pointclouds = obstacle_pointclouds
        elif map_pointclouds is not None and obstacle_pointclouds is not None:
            pointclouds = np.vstack((map_pointclouds, obstacle_pointclouds))
        else:
            return

        pointCloud2 = self.GeneratePointCloud2(pointclouds, self.publish_frame)
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

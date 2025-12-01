import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Header
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import math
from visualization_msgs.msg import Marker, MarkerArray
from custom_msgs.msg import DeltaOdom, Belief
from mappeate_y_ubicate.robot_functions import RobotFunctions, RESOLUTION, OX, OY, HEIGHT, WIDTH

def yaw_to_quaternion(yaw):
    """Convert a yaw angle (in radians) into a Quaternion message."""
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q

class FastSLAMNode(Node):
    def __init__(self):
        super().__init__('fast_slam_node')

        self.declare_parameter("num_particles", 10)
        num_particles = self.get_parameter("num_particles").get_parameter_value().integer_value

        self.delta_sub = self.create_subscription(
            DeltaOdom, "/delta", self.delta_callback, 10
        )

        self.subscription_scan = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            "/map",
            map_qos,
        )

        self.particle_path_pub = self.create_publisher(Path, "/particle_robot_path", 10)
        self.particle_path_msg = Path()
        self.particle_path_msg.header.frame_id = "map"

        self.robot = RobotFunctions(num_particles)
        self.map_resolution = RESOLUTION
        self.map_width = WIDTH
        self.map_height = HEIGHT
        self.map_origin_x = OX
        self.map_origin_y = OY

    def delta_callback(self, msg: DeltaOdom):
        odom = {
            'r1': msg.dr1,
            'r2': msg.dr2,
            't': msg.dt
        }
        self.robot.move_particles(odom)\

        if not (odom['r1'] == 0 and odom['r2'] == 0 and odom['t'] == 0):
            print(f"r1: {odom['r1']}, r2: {odom['r2']}, t: {odom['t']}")
        
        self.plot_particle_and_map()

    def scan_callback(self, msg: LaserScan):
        self.robot.update(msg)

    def plot_particle_and_map(self):
        best_particle = self.robot.get_best_particle()
        selected_state = np.array([best_particle.x, best_particle.y, best_particle.orientation]) 

        mean_x = selected_state[0]
        mean_y = selected_state[1]

        # Handle circular mean for angle
        cos_t = np.cos(selected_state[2])
        sin_t = np.sin(selected_state[2])
        mean_theta = np.arctan2(sin_t, cos_t)

        # Build PoseStamped for the mean particle
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = float(mean_x)
        pose_stamped.pose.position.y = float(mean_y)
        pose_stamped.pose.position.z = 0.0

        # Convert yaw to quaternion
        q = yaw_to_quaternion(mean_theta)
        pose_stamped.pose.orientation = q

        # Append to path and publish
        self.particle_path_msg.poses.append(pose_stamped)
        self.particle_path_msg.header.stamp = self.get_clock().now().to_msg()
        self.particle_path_pub.publish(self.particle_path_msg)

        #Publish map
        msg = self.logoddsgrid_to_occupancygrid(best_particle.grid)
        self.map_publisher.publish(msg)

    def logoddsgrid_to_occupancygrid(self, logodds_grid):
        p = 1.0 / (1.0 + np.exp(-logodds_grid))

        data = (p * 100).astype(np.uint8)
        data_flat = data.flatten(order='C').tolist()

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = float(self.map_resolution)
        msg.info.width = self.map_width
        msg.info.height = self.map_height

        msg.info.origin = Pose()
        msg.info.origin.position.x = float(self.map_origin_x)
        msg.info.origin.position.y = float(self.map_origin_y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation = Quaternion(x = 0.0, y = 0.0, z = 0.0, w = 1.0)

        msg.data = data_flat

        return msg

    
def main(args=None):
    rclpy.init(args=args)
    node = FastSLAMNode()
    
    try:
        rclpy.spin(node)  # Keep ROS 2 running
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
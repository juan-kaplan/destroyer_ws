import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path, GridCells
from geometry_msgs.msg import PoseStamped, Point, Twist
from sensor_msgs.msg import LaserScan
from enum import Enum, auto
import numpy as np
import heapq
import math

class State(Enum):
    IDLE = auto()
    PLANNING = auto()
    PURSUIT = auto()
    OBSTACLE = auto()

class PursuitConfig:
        LOOKAHEAD_DIST = 0.15
        GOAL_TOLERANCE = 0.05
        LINEAR_VEL = 0.05
        ROTATION_SPEED = 0.5
        HEADING_TOLERANCE = 0.4
        MAX_ANGULAR_VEL = 1.5
        GOAL_ANGLE_TOLERANCE = 0.1
        OBSTACLE_DETECT_DIST = 0.5
        OBSTACLE_MAP_DIST = 0.6
        SCAN_ANGLE_WIDTH = math.pi / 6
        INFLATION_RADIUS = 3
        OBSTACLE_SIGMA = 0.3

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self.state = State.IDLE

        self.static_map = None
        self.likelihood_map = None
        self.combined_map = None
        self.dynamic_map = None

        self.goal_pose = None
        self.current_pose = None

        self.maps_ready = False

        self.front_scan = None
        self.cfg = PursuitConfig()

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.sub_static = self.create_subscription(
            OccupancyGrid, '/map', self.static_map_callback, map_qos)

        self.sub_likelihood = self.create_subscription(
            OccupancyGrid, '/likelihood_map', self.likelihood_map_callback, map_qos)

        self.sub_goal = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.sub_pose = self.create_subscription(
            PoseStamped, '/robot_pose', self.pose_callback, 10)
        
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.pub_costmap = self.create_publisher(OccupancyGrid, '/fused_costmap', map_qos)
        self.pub_path = self.create_publisher(Path, '/global_plan', 10)
        self.pub_path_grid = self.create_publisher(GridCells, '/path_grid', 10)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Navigator Initialized in IDLE state.")

    def control_loop(self):
        if self.state == State.IDLE:
            self.run_idle_state()

        elif self.state == State.PLANNING:
            self.run_planning_state()

        elif self.state == State.PURSUIT:
            self.run_pursuit_state()

        elif self.state == State.OBSTACLE:
            self.run_obstacle_state()

    def run_idle_state(self):
        # 1. Map Initialization
        if self.static_map is None:
            self.get_logger().info("Waiting for /map...", throttle_duration_sec=2.0)
            return

        if self.likelihood_map is None:
            self.get_logger().info("Waiting for /likelihood_map...", throttle_duration_sec=2.0)
            return
            
        # Initialize the combined map if not done yet
        if not self.maps_ready:
            self.get_logger().info("Maps received. Fusing Static + Likelihood...")
            self.init_combined_map()
            self.maps_ready = True

        # 2. Navigation Prerequisites
        if self.current_pose is None:
            self.get_logger().info("Waiting for /robot_pose (Localization)...", throttle_duration_sec=2.0)
            return

        if self.goal_pose is None:
            self.get_logger().info("Waiting for /goal_pose...", throttle_duration_sec=2.0)
            return

        # 3. Transition
        self.get_logger().info("IDLE: All systems ready. Transitioning to PLANNING.")
        self.state = State.PLANNING

    def run_planning_state(self):
        start_world = (self.current_pose.pose.position.x, self.current_pose.pose.position.y)
        goal_world = (self.goal_pose.pose.position.x, self.goal_pose.pose.position.y)
        
        start_grid = self.world_to_grid(start_world)
        goal_grid = self.world_to_grid(goal_world)

        if self.get_cost(*start_grid) >= 100 or self.get_cost(*goal_grid) >= 100:
            self.get_logger().warn("PLANNING: Start or Goal is inside an obstacle!")
            self.state = State.IDLE
            return

        path_grid = self.astar_search(start_grid, goal_grid)

        if path_grid:
            self.get_logger().info(f"PLANNING: Path found with {len(path_grid)} steps.")

            self.current_path = [self.grid_to_world(node) for node in path_grid]

            self.publish_path(self.current_path)
            self.publish_path_grid(path_grid)

            self.state = State.PURSUIT
            self.get_logger().info("PLANNING: Path found. Transitioning to PURSUIT.")
        
        else:
            self.get_logger().error("PLANNING: No path found.")
            self.state = State.IDLE
            return
    
    def run_pursuit_state(self):
        if not self.current_path or not self.current_pose:
            self.state = State.IDLE
            return

        rx, ry, ryaw = self.get_robot_state()

        if self.check_goal_reached(rx, ry):
            return

        self.prune_path(rx, ry)
        if not self.current_path:
            self.state = State.IDLE
            return

        target_point = self.get_lookahead_point(rx, ry, self.cfg.LOOKAHEAD_DIST)
        cmd = self.compute_velocity_command(target_point, rx, ry, ryaw)

        if cmd.linear.x > 0.0:
            if self.check_collision_risk():
                self.stop_robot()
                self.get_logger().info("Transitioning to OBSTACLE.")
                self.state = State.OBSTACLE
                return

        self.pub_vel.publish(cmd)

    def run_obstacle_state(self):
        if self.front_scan is None or self.current_pose is None:
            self.state = State.IDLE
            return

        rx, ry, ryaw = self.get_robot_state()

        ranges = self.front_scan['ranges']
        angles = self.front_scan['angles']

        valid = ranges < self.cfg.OBSTACLE_MAP_DIST

        ranges = ranges[valid]
        angles = angles[valid]

        local_x = ranges * np.cos(angles)
        local_y = ranges * np.sin(angles)

        world_x = rx + (local_x * np.cos(ryaw) - local_y * np.sin(ryaw))
        world_y = ry + (local_x * np.sin(ryaw) + local_y * np.cos(ryaw))

        obstacles_marked = 0

        if self.dynamic_map is None:
             self.init_combined_map()

        for wx, wy in zip(world_x, world_y):
            gx, gy = self.world_to_grid((wx, wy))
            if gx < 0 or gx >= self.map_info.width or gy < 0 or gy >= self.map_info.height:
                continue
            
            self.dynamic_map[gy, gx] = 100
            obstacles_marked += 1

            r = self.cfg.INFLATION_RADIUS
            sigma_sq = self.cfg.OBSTACLE_SIGMA ** 2
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < self.map_info.width and 0 <= ny < self.map_info.height:
                        dist_sq = (dx * self.map_info.resolution)**2 + (dy * self.map_info.resolution)**2
                        cost = 100.0 * math.exp(-dist_sq / (2.0 * sigma_sq))

                        old_cost = self.dynamic_map[ny, nx]
                        self.dynamic_map[ny, nx] = max(old_cost, int(cost))

        self.get_logger().info(f"OBSTACLE: Marked {obstacles_marked} obstacles.")
        if self.combined_map is not None:
             debug_view = self.combined_map + self.dynamic_map
             self.publish_debug_map(debug_view)

        self.state = State.PLANNING
        self.get_logger().info("OBSTACLE: Transitioning to PLANNING.")
        return
        
    def astar_search(self, start, goal):
        open_list = []
        heapq.heappush(open_list, (0, start[0], start[1]))

        came_from = {}
        g_score = {start: 0}

        neighbors = [
            (0, 1, 1.0), (0, -1, 1.0), (1, 0, 1.0), (-1, 0, 1.0),  # Straight
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414) # Diagonal
        ]

        while open_list:
            current_f, cx, cy = heapq.heappop(open_list)
            current = cx, cy

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for dx, dy, move_cost in neighbors:
                neighbor = cx + dx, cy + dy
                cell_cost = self.get_cost(*neighbor)

                if cell_cost == float('inf') or cell_cost >= 100:
                    continue

                terrain_cost = cell_cost * 0.1
                new_g = g_score[current] + move_cost + terrain_cost

                if neighbor not in g_score or new_g < g_score[neighbor]:
                    g_score[neighbor] = new_g
                    f_score = new_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score, neighbor[0], neighbor[1]))
                    came_from[neighbor] = current
        
        return None

    def heuristic(self, a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
                
    def init_combined_map(self):
        static_grid = np.array(self.static_map.data, dtype=np.int16).reshape(
            (self.map_info.height, self.map_info.width))

        like_grid = np.array(self.likelihood_map.data, dtype=np.int16).reshape(
            (self.map_info.height, self.map_info.width))

        self.dynamic_map = np.zeros_like(static_grid, dtype=np.int16)

        static_grid[static_grid == -1] = 100

        self.combined_map = static_grid + like_grid

        self.get_logger().info(f"Maps fused. Shape: {self.combined_map.shape}")
        self.publish_debug_map(self.combined_map)

    def get_cost(self, x, y):
        if not (0 <= x < self.map_info.width and 0 <= y < self.map_info.height):
            return float('inf')

        base = self.combined_map[y, x]

        dyn = self.dynamic_map[y, x]

        return base + dyn

    def stop_robot(self):
        """ Publishes a zero velocity command to stop immediately """
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub_vel.publish(cmd)

    def prune_path(self, rx, ry):
        """Removes path points the robot has already passed."""
        closest_idx = 0
        min_dist = float('inf')
        
        search_range = min(len(self.current_path), 50)
        
        for i in range(search_range):
            dist = math.hypot(self.current_path[i][0] - rx, self.current_path[i][1] - ry)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        self.current_path = self.current_path[closest_idx:]

    def get_lookahead_point(self, rx, ry, lookahead_dist):
        target = self.current_path[0]
        
        for p in self.current_path:
            dist = math.hypot(p[0] - rx, p[1] - ry)
            if dist > lookahead_dist:
                target = p
                break
        return target

    def check_goal_reached(self, rx, ry):
        final_pt = self.current_path[-1]
        dist_to_goal = math.hypot(final_pt[0] - rx, final_pt[1] - ry)

        if dist_to_goal <= self.cfg.GOAL_TOLERANCE:
            if self.reach_goal_angle(tol=self.cfg.GOAL_ANGLE_TOLERANCE):
                self.get_logger().info("PURSUIT: Goal reached & Aligned.")
                self.stop_robot()
                self.current_path = []
                self.goal_pose = None
                self.state = State.IDLE
            return True
        
        return False

    def reach_goal_angle(self, tol=0.1):
        if self.goal_pose is None or self.current_pose is None:
            return True
        
        current_yaw = self.get_yaw_from_pose(self.current_pose.pose)
        goal_yaw = self.get_yaw_from_pose(self.goal_pose.pose)
        
        yaw_error = goal_yaw - current_yaw

        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi

        if abs(yaw_error) <= tol:
            return True
        
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 1.0 * yaw_error 
        cmd.angular.z = max(min(cmd.angular.z, 1.0), -1.0)

        self.pub_vel.publish(cmd)

        return False

    def compute_velocity_command(self, target, rx, ry, ryaw):
        tx, ty = target
        dx = tx - rx
        dy = ty - ry

        target_heading = math.atan2(dy, dx)
        heading_error = self.normalize_angle(target_heading - ryaw)

        cmd = Twist()

        # Rotate in Place
        if abs(heading_error) > self.cfg.HEADING_TOLERANCE:
            cmd.linear.x = 0.0
            cmd.angular.z = 2.0 * heading_error
            cmd.angular.z = max(min(cmd.angular.z, self.cfg.ROTATION_SPEED), -self.cfg.ROTATION_SPEED)
        
        # Pure Pursuit
        else:
            local_x = dx * math.cos(-ryaw) - dy * math.sin(-ryaw)
            local_y = dx * math.sin(-ryaw) + dy * math.cos(-ryaw)

            L = math.hypot(local_x, local_y)
            
            curvature = 0.0
            if L > 0.01:
                curvature = (2.0 * local_y) / (L ** 2)

            cmd.linear.x = self.cfg.LINEAR_VEL
            cmd.angular.z = curvature * self.cfg.LINEAR_VEL
            
            cmd.angular.z = max(min(cmd.angular.z, self.cfg.MAX_ANGULAR_VEL), -self.cfg.MAX_ANGULAR_VEL)

        return cmd

    def check_collision_risk(self):
        if self.front_scan is None or len(self.front_scan['ranges']) == 0:
            return False

        ranges = self.front_scan['ranges']
        angles = self.front_scan['angles']

        danger_mask = ranges < self.cfg.OBSTACLE_DETECT_DIST
        if not np.any(danger_mask):
            return False
        
        ranges = ranges[danger_mask]
        angles = angles[danger_mask]

        rx, ry, ryaw = self.get_robot_state()
        
        local_x = ranges * np.cos(angles)
        local_y = ranges * np.sin(angles)
        
        world_x = rx + (local_x * np.cos(ryaw) - local_y * np.sin(ryaw))
        world_y = ry + (local_x * np.sin(ryaw) + local_y * np.cos(ryaw))

        unknown_obstacles_count = 0

        for wx, wy in zip(world_x, world_y):
            gx, gy = self.world_to_grid((wx, wy))
            
            # Check bounds
            if not (0 <= gx < self.map_info.width and 0 <= gy < self.map_info.height):
                continue

            current_cost = self.get_cost(gx, gy)
            if current_cost < 50:
                unknown_obstacles_count += 1

        if unknown_obstacles_count > 3:
            self.get_logger().info(f"COLLISION RISK! Detected {unknown_obstacles_count} unmapped points.")
            return True

        return False

    def get_robot_state(self):
        pose = self.current_pose.pose
        return (pose.position.x, pose.position.y, self.get_yaw_from_pose(pose))

    def static_map_callback(self, msg):
        self.static_map = msg
        self.map_info = msg.info

    def likelihood_map_callback(self, msg):
        self.likelihood_map = msg

    def goal_callback(self, msg):
        self.goal_pose = msg
        self.get_logger().info(f"Received Goal: {msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}")

        if self.state != State.IDLE:
             self.get_logger().info("New goal received during operation. Replanning.")
             self.state = State.PLANNING

    def pose_callback(self, msg):
        self.current_pose = msg

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        angles = np.arctan2(np.sin(angles), np.cos(angles))

        cone_mask = (angles > -self.cfg.SCAN_ANGLE_WIDTH) & \
                    (angles < self.cfg.SCAN_ANGLE_WIDTH)
        valid_mask = cone_mask & (ranges > msg.range_min)

        self.front_scan = {
            'ranges': ranges[valid_mask],
            'angles': angles[valid_mask]
        }

    def publish_debug_map(self, grid_data):
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = self.map_info
        
        display_grid = np.clip(grid_data, 0, 100).astype(np.int8)
        msg.data = display_grid.flatten().tolist()
        self.pub_costmap.publish(msg)

    def publish_path(self, path_points):
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        for p in path_points:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            msg.poses.append(pose)

        self.pub_path.publish(msg)

    def publish_path_grid(self, path_nodes):
        msg = GridCells()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.cell_width = self.map_info.resolution
        msg.cell_height = self.map_info.resolution

        for node in path_nodes:
            wx, wy = self.grid_to_world(node)

            p = Point()
            p.x = wx
            p.y = wy
            msg.cells.append(p)

        self.pub_path_grid.publish(msg)

    def world_to_grid(self, world_pos):
        wx, wy = world_pos
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        res = self.map_info.resolution
        
        gx = int((wx - ox) / res)
        gy = int((wy - oy) / res)
        return (gx, gy)

    def grid_to_world(self, grid_pos):
        gx, gy = grid_pos
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        res = self.map_info.resolution
        
        wx = (gx * res) + ox + (res / 2.0) # Center of cell
        wy = (gy * res) + oy + (res / 2.0)
        return (wx, wy)

    def get_yaw_from_pose(self, pose):
        """ Extract yaw (rotation around Z) from a Quaternion """
        q = pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle
    

def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
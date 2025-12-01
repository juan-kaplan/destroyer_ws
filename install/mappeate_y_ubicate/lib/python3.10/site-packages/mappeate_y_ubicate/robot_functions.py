import numpy as np
import random
import copy
from scipy.special import logsumexp 

HEIGHT = 200      # 50 meters / 0.1 resolution
WIDTH = 200
RESOLUTION = 0.1  # 10 cm per pixel
OX = - (WIDTH * RESOLUTION) / 2.0   # -25.0
OY = - (HEIGHT * RESOLUTION) / 2.0  # -25.0   

# probability parameters
P_OCC  = 0.7    # probability of occupancy when hit
P_FREE = 0.3    # probability of occupancy for free cells (less than 0.5)

# log-odds increments (l0 = 0 => p=0.5)
L_OCC  = np.log(P_OCC / (1.0 - P_OCC))
L_FREE = np.log(P_FREE / (1.0 - P_FREE))

# optional clamping to avoid extreme values
L_MIN  = -5.0
L_MAX  =  5.0


class particle():

    def __init__(self):
        self.x = 0  # initial x position
        self.y = 0 # initial y position
        self.orientation = 0 # initial orientation
        self.log_weight = 0.0 
        self.grid = np.zeros((HEIGHT, WIDTH))

    def set(self, new_x, new_y, new_orientation):
        '''
        set: sets a robot coordinate, including x, y and orientation
        '''
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)

    def move_odom(self,odom,noise):
        '''
        move_odom: Takes in Odometry data and moves the robot based on the odometry data
        
        Devuelve una particula (del robot) actualizada
        '''      
        dist  = odom['t']       
        delta_rot1  = odom['r1']
        delta_rot2 = odom['r2']
        alpha1, alpha2, alpha3, alpha4 = noise

        rot1_hat = delta_rot1 + np.random.normal(0, alpha1 * abs(delta_rot1) + alpha2 * dist)
        trans_hat = dist + np.random.normal(0, alpha3 * dist + alpha4 * (abs(delta_rot1) + abs(delta_rot2)))
        rot2_hat = delta_rot2 + np.random.normal(0, alpha1 * abs(delta_rot2) + alpha2 * trans_hat)

        x_new = self.x + trans_hat * np.cos(self.orientation + rot1_hat)
        y_new = self.y + trans_hat * np.sin(self.orientation + rot1_hat)
        theta_new = (self.orientation + rot1_hat + rot2_hat) 

        self.set(x_new, y_new, theta_new)

    def update_grid(self, scan):
        ranges = scan.ranges
        range_min = scan.range_min
        range_max = scan.range_max
        angle_min = scan.angle_min
        angle_max = scan.angle_max
        angle_increment = scan.angle_increment

        H, W = self.grid.shape
        robot_i, robot_j = self.coordinate_to_grid(self.x, self.y, OX, OY, RESOLUTION)
        print(f'robot_i: {robot_i}, robot_j: {robot_j}')
        if not (0 <= robot_i < H and 0 <= robot_j < W):
            # robot outside map → nothing to update
            return

        points = _scan_reference(
            ranges, range_min, range_max,
            angle_min, angle_max, angle_increment,
            [self.x, self.y, self.orientation],
        )

        if points.size == 0:
            return

        xs, ys = points

        beam_step = 20
        xs, ys = xs[::beam_step], ys[::beam_step]

        end_i, end_j = self.coordinate_to_grid(xs, ys, OX, OY, RESOLUTION)

        for ci, cj in zip(end_i, end_j):
            if not (0 <= ci < H and 0 <= cj < W):
                continue

            ray_cells = self._bresenham(robot_i, robot_j, ci, cj)
            for ri, rj in ray_cells[:-1]:
                if 0 <= ri < H and 0 <= rj < W:
                    self.grid[ri, rj] += L_FREE

            end_i, end_j = ray_cells[-1]
            if 0 <= end_i < H and 0 <= end_j < W:
                self.grid[end_i, end_j] += L_OCC

        self.grid = np.clip(self.grid, L_MIN, L_MAX)

    def update_weight(self, scan):
        ranges = scan.ranges
        range_min = scan.range_min
        range_max = scan.range_max
        angle_min = scan.angle_min
        angle_max = scan.angle_max
        angle_increment = scan.angle_increment

        points = _scan_reference(ranges, range_min, range_max, angle_min, angle_max, angle_increment, [self.x, self.y, self.orientation])
        if points.size == 0:
            self.log_weight = -np.inf
            return
        
        row, col = self.coordinate_to_grid(points[0], points[1], OX, OY, RESOLUTION)
        H, W = self.grid.shape
        inb = (row >= 0) & (row < H) & (col >= 0) & (col < W)
        if not np.any(inb):
            self.log_weight = -np.inf
            return

        logodds = self.grid[row[inb], col[inb]]
        p = 1.0 / (1.0 + np.exp(-logodds))
        p = np.clip(p, 1e-6, 1.0 - 1e-6)
        self.log_weight = np.mean(np.log(p))

    def coordinate_to_grid(self, x, y, ox, oy, res):
        x_arr = np.asarray(x)
        y_arr = np.asarray(y)
        j = np.floor((x_arr - ox) / res).astype(int)
        i = np.floor((y_arr - oy) / res).astype(int)
        return i, j

    def _bresenham(self, i0, j0, i1, j1):
        """
        Bresenham's line algorithm from (i0, j0) to (i1, j1).
        Returns list of (i, j) including both endpoints.
        Indices are ints: i = row (y), j = col (x).
        """
        cells = []

        di = abs(i1 - i0)
        dj = abs(j1 - j0)
        si = 1 if i0 < i1 else -1
        sj = 1 if j0 < j1 else -1

        err = di - dj
        i, j = i0, j0

        while True:
            cells.append((i, j))
            if i == i1 and j == j1:
                break
            e2 = 2 * err
            if e2 > -dj:
                err -= dj
                i += si
            if e2 < di:
                err += di
                j += sj

        return cells

            

class RobotFunctions:

    def __init__(self, num_particles=0):
        if num_particles != 0:
            self.num_particles = num_particles
            self.particles = [particle() for _ in range(self.num_particles)]
            self.weights = np.ones(self.num_particles) / self.num_particles
            self.resample_ratio = 0.5

    def get_weights(self,):
        return self.weights
    
    def get_particle_states(self,):
        samples = np.array([[p.x, p.y, p.orientation] for p in self.particles])
        return samples
    
    def move_particles(self, deltas):
        for part in self.particles:
            part.move_odom(deltas, [0.05, 0.05, 0.001, 0.001])
    
    def get_best_particle(self):
        idx = int(np.argmax(self.weights))
        return self.particles[idx]

    def update(self, scan):
        N_eff = self.update_particles(scan)
        for p in self.particles:
            p.update_grid(scan)

    def update_particles(self, scan):
        for part in self.particles:
            part.update_weight(scan)

        log_ws = np.array([p.log_weight for p in self.particles])
        if np.isneginf(log_ws).all():
            # no particle had any valid measurement -> keep uniform
            self.weights = np.ones(self.num_particles, dtype=float) / self.num_particles
        else:
            # Normalize log-weights → weights in a numerically stable way
            # log_ws can contain -inf for some particles; logsumexp handles that.
            log_ws -= logsumexp(log_ws)      # now log(sum(exp(log_ws))) = 0
            self.weights = np.exp(log_ws)    # sum_i weights[i] = 1

        N_eff = 1.0 / np.sum(self.weights ** 2)
        N_threshold = self.resample_ratio * self.num_particles
        if N_eff < N_threshold:
            indices = self.systematic_resample(self.weights, self.num_particles)
            self.particles = [copy.deepcopy(self.particles[i]) for i in indices]

            self.weights = np.ones(self.num_particles, dtype=float) / self.num_particles
            for p in self.particles:
                p.log_weight = 0.0

        return N_eff

    def systematic_resample(self, weights: np.array, num_samples: int):
        cumulative_sum = np.cumsum(weights)

        rng = np.random.default_rng()
        u0 = rng.random() / num_samples

        points = u0 + np.arange(num_samples) / num_samples
        idx = np.searchsorted(cumulative_sum, points)
        return idx

def _scan_reference(ranges, range_min, range_max, angle_min, angle_max, angle_increment, last_odom):
        '''
        Scan Reference recibe:
            - ranges: lista rangos del escáner láser
            - range_min: rango mínimo del escáner
            - range_max: rango máximo del escáner
            - angle_min: ángulo mínimo del escáner
            - angle_max: ángulo máximo del escáner
            - angle_increment: incremento de ángulo del escáner
            - last_odom: última odometría [tx, ty, theta]
        Devuelve puntos en el mapa transformados a coordenadas globales donde 
            - points_map[0]: coordenadas x
            - points_map[1]: coordenadas y
        '''
        LIDAR = (0.0, 0.0, 0.0)
        ranges = np.asarray(ranges, float)
        angles = angle_min + np.arange(len(ranges)) * angle_increment

        eps = 1e-3
        valid = np.isfinite(ranges) & (ranges > range_min) & (ranges < (range_max - eps))
        if not np.any(valid):
            return np.empty((2,0))  # no valid hits

        r = ranges[valid]
        a = angles[valid]
        pts_cart = np.column_stack((r*np.cos(a), r*np.sin(a), np.ones_like(r)))

        T_lidar_robot = _T(*LIDAR)
        T_robot_world = _T(*last_odom)
        T_lidar_world = T_robot_world @ T_lidar_robot

        pts_world = (T_lidar_world @ pts_cart.T)[:2, :]
        return pts_world

def _T( x, y, theta):
    """Create a transformation matrix for translation and rotation."""
    return np.array([[np.cos(theta), -np.sin(theta), x],
                    [np.sin(theta), np.cos(theta), y],
                    [0, 0, 1]])
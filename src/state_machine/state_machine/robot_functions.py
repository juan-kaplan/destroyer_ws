import numpy as np
import random
import copy
from scipy.special import logsumexp 

class particle():

    def __init__(self):
        self.x = (random.random()-0.5)*2  # initial x position
        self.y = (random.random()-0.5)*2 # initial y position
        self.orientation = random.uniform(-np.pi,np.pi) # initial orientation
        self.weight = 1.0

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

    def set_weight(self, weight):
        '''
        set_weights: sets the weight of the particles
        '''
        #noise parameters
        self.weight  = float(weight)

class RobotFunctions:

    def __init__(self, num_particles=0):
        if num_particles != 0:
            self.num_particles = num_particles
            self.particles = []
            for _ in range(self.num_particles):
                self.particles.append(particle())

            self.weights = np.ones(self.num_particles) / self.num_particles

    def get_weights(self,):
        return self.weights
    
    def get_particle_states(self,):
        samples = np.array([[p.x, p.y, p.orientation] for p in self.particles])
        return samples
    
    def move_particles(self, deltas):
        for part in self.particles:
            part.move_odom(deltas, [10.0, 10.0, 0.1, 0.1])
    
    def get_selected_state(self,):
        '''
        Esta funcion debe devolver lo que ustedes consideran como la posición del robot segun las particulas.
        Queda a su criterio como la obtienen en base a las particulas.
        '''
        x_values = np.array([p.x for p in self.particles])
        y_values = np.array([p.y for p in self.particles])
        orientations = np.array([p.orientation for p in self.particles])
        weights = np.array([p.weight for p in self.particles])

        mean_x = np.average(x_values, weights=weights)
        mean_y = np.average(y_values, weights=weights)

        mean_sin = np.average(np.sin(orientations), weights=weights)
        mean_cos = np.average(np.cos(orientations), weights=weights)
        mean_orientation = np.arctan2(mean_sin, mean_cos)
        return [mean_x, mean_y, mean_orientation]

    def update_particles(self, data, map_data, grid):
        '''
        La funcion update_particles se llamará cada vez que se recibe data del LIDAR
        Esta funcion toma:
            data: datos del lidar en formato scan (Ver documentacion de ROS sobre tipo de dato LaserScan).
                  Pueden aprovechar la funcion scan_refererence del TP1 para convertir los datos crudos en
                  posiciones globales calculadas
            map_data: Es el mensaje crudo del mapa de likelihood. Pueden consultar la documentacion de ROS
                      sobre tipos de dato OccupancyGrid.
            grid: Es la representación como matriz de numpy del mapa de likelihood. 
                  Importante:
                    - La grilla se indexa como grid[y, x], primero fila (eje Y) y luego columna (eje X).
                    - La celda (0,0) corresponde a la esquina inferior izquierda del mapa en coordenadas de ROS.
        
        Esta funcion debe tomar toda esta data y actualizar el valor de probabilidad (weight) de cada partícula
        En base a eso debe resamplear las partículas. Tenga cuidado al resamplear de hacer un deepcopy para que 
        no sean el mismo objeto de python
        '''
        ranges = data.ranges
        range_min = data.range_min
        range_max = data.range_max
        angle_min = data.angle_min
        angle_max = data.angle_max
        angle_increment = data.angle_increment

        res = map_data.info.resolution
        ox = map_data.info.origin.position.x
        oy = map_data.info.origin.position.y

        height, width = map_data.info.height, map_data.info.width

        particle_states = self.get_particle_states()
        log_weights = np.zeros(self.num_particles)

        for i in range(self.num_particles):
            points = self.scan_reference(ranges, range_min, range_max, angle_min, angle_max, angle_increment, particle_states[i])
            if points.size == 0:      # shape (2, 0)
                continue

            row, col = self.coordinate_to_grid(points[0], points[1], ox, oy, res)
            inb = (row >= 0) & (row < height) & (col >= 0) & (col < width)
            if not np.any(inb):
                continue

            cells = grid[row[inb], col[inb]]

            p = np.clip(cells / 100.0, 1e-6, 1)
            log_weights[i] = np.mean(np.log(p))

        self.weights = np.exp(log_weights - logsumexp(log_weights))
        self.weights /= np.sum(self.weights)

        for i in range(self.num_particles):
            self.particles[i].set_weight(self.weights[i])

        idx = self.systematic_resample(self.weights, self.num_particles)
        self.particles = [copy.deepcopy(self.particles[i]) for i in idx]
        self.weights = np.ones(self.num_particles) / self.num_particles

    def systematic_resample(self, weights: np.array, num_samples: int):
        cumulative_sum = np.cumsum(weights)

        rng = np.random.default_rng()
        u0 = rng.random() / num_samples

        points = u0 + np.arange(num_samples) / num_samples
        idx = np.searchsorted(cumulative_sum, points)
        return idx


    def coordinate_to_grid(self, x, y, ox, oy, res):
        '''
        Convert world coordinates (x, y) to grid coordinates (i, j).
        Supports both scalar and NumPy array inputs.
        '''
        j = ((x - ox) / res).astype(int)
        i = ((y - oy) / res).astype(int)
        return i, j


    def scan_reference(self, ranges, range_min, range_max, angle_min, angle_max, angle_increment, last_odom):
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

        T_lidar_robot = self._T(*LIDAR)
        T_robot_world = self._T(*last_odom)
        T_lidar_world = T_robot_world @ T_lidar_robot

        pts_world = (T_lidar_world @ pts_cart.T)[:2, :]
        return pts_world

    def _T(self, x, y, theta):
        """Create a transformation matrix for translation and rotation."""
        return np.array([[np.cos(theta), -np.sin(theta), x],
                        [np.sin(theta), np.cos(theta), y],
                        [0, 0, 1]])
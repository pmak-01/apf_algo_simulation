import numpy as np

class Robot:
    '''Represents the differential drive robot, including its navigation algorithms'''

    def __init__(self, x_lim, y_lim, tolerance_radii: tuple[int, int], goal_pos: list[float], robot_pos: list[float]):
        # self.pos = np.array(pos).reshape(2, 1)
        # self.goal_pos = np.array(goal_pos).reshape(2, 1)
        self.robot_pos = np.array(robot_pos, dtype=float).reshape(2, 1)
        self.goal_pos = np.array(goal_pos, dtype=float).reshape(2, 1)
        self.x_min, self.x_max = x_lim
        self.y_min, self.y_max = y_lim
        self.X = x_lim[1] - x_lim[0] # width (x) of map
        self.Y = y_lim[1] - y_lim[0] # 'height' (y) of map
        self.sensor_readings = np.array([[[], []]])
        '''sensor readings are a column vector of x,y coordinates of detected points'''
        self.avoid_radius, self.goal_radius = tolerance_radii
        self.step_size = 20
        self.point_idx = 0
        '''Index for incoming sensor readings. Resets to 0 after 99. So only 100 sensor readings can be stored at a time'''
        
    def _dist(self, pt_1: np.ndarray, pt_2: np.ndarray) -> float:
        return np.sqrt(sum((pt_1-pt_2)**2))

    def _calc_all_dist(self, arr) -> np.ndarray:
            reshaped_robot_pos = np.expand_dims(self.robot_pos, axis=0)
            return np.sqrt(np.sum((arr-self.robot_pos)**2, axis=1))[:,0]

    def update_sensor_readings(self, new_readings):
        '''
        `new_readings` is a column vector of Cartesian coordinates of detected points.
        Assumes that data points have been cleaned i.e. are within the map
        '''

        if len(self.sensor_readings[0]) + len(new_readings) > 100:
            space_required = int((len(self.sensor_readings[0]) + len(new_readings))-100)
            print(f"Space required: {space_required}")
            self.sensor_readings = self.sensor_readings[:, space_required:].copy()

        new = np.array([np.array([[reading[0]],
                                  [reading[1]]]) for reading in new_readings])

        # x = [reading[0] for reading in new_readings]
        # y = [reading[1] for reading in new_readings]
        # new = np.array([x, y])
        try:
            self.sensor_readings = np.append(self.sensor_readings, new, axis=0)
        except ValueError:
            self.sensor_readings = new.copy()

    def get_robot_pos(self) -> list[float]:
        return self.robot_pos.reshape(1,2).tolist()[0]

    def is_at_goal(self) -> bool:
        return self._dist(self.goal_pos, self.robot_pos) <= self.goal_radius

    def calculate_next_move(self):
        displacement = 0.65*self._calc_goal_attraction() + 0.3*(self._calc_wall_x_repulsion() + self._calc_wall_y_repulsion()) + 0.05*np.sum(self._calc_obstacle_repulsion(), axis=0)

        displacement *= self.step_size/np.linalg.vector_norm(displacement)

        self.robot_pos += displacement

    def _calc_goal_attraction(self) -> np.ndarray:
        '''
        Returns a normalized column vector denoting attractive force,
        directed towards goal
        '''
        g = self.goal_pos - self.robot_pos
        g *= 1./np.linalg.vector_norm(g)
        return g

    def _calc_wall_x_repulsion(self) -> np.ndarray:
        '''
        Returns a normalized column vector denoting repulsive,
        from the left or right walls, acting parallel to x-axis
        '''
        if self.robot_pos[0][0]<self.X/2.:
            w_x = np.array([[self.robot_pos[0][0]],
                            [0.]])
        else:
            w_x = np.array([[self.robot_pos[0][0]-self.X],
                            [0]])
        w_x *= (self.X/2. - np.linalg.vector_norm(w_x))/(self.X/2. * np.linalg.vector_norm(w_x))
        return w_x

    def _calc_wall_y_repulsion(self) -> np.ndarray:
        '''
        Returns a normalized column vector denoting repulsive,
        from the upper or lower walls, acting parallel to y-axis
        '''
        if self.robot_pos[1][0]<self.Y/2.:
            w_y = np.array([[0],
                            [self.robot_pos[1][0]]])
        else:
            w_y = np.array([[0],
                            [self.robot_pos[1][0] - self.Y]])
        w_y *= (self.Y/2. - np.linalg.vector_norm(w_y))/(self.Y/2. * np.linalg.vector_norm(w_y))
        return w_y

    def _calc_obstacle_repulsion(self) -> np.ndarray:
        '''
        Returns a vector of normalized vectors that denote repulsion from each 
        obstacle point within the avoidance radius
        '''
        valid_points = self.sensor_readings[self._calc_all_dist(self.sensor_readings) <= self.avoid_radius]
        repul = (self.robot_pos - valid_points)
        repul *= (self.avoid_radius - np.linalg.norm(repul, axis=1, keepdims=True)) / (self.avoid_radius * np.linalg.norm(repul, axis=1, keepdims=True))
        # print(self.avoid_radius - np.linalg.matrix_norm(repul))
        # print(self.avoid_radius*np.linalg.matrix_norm(repul))
        # t = (self.avoid_radius - np.linalg.matrix_norm(repul))/(self.avoid_radius*np.linalg.matrix_norm(repul))
        # print(repul.shape, t.shape)
        # repul = repul*t
        # repul *= np.array([[t],[t]])
        return repul
    

if __name__ == '__main__':
    robot = Robot((0, 750), (0, 450), (300, 100), [700, 350], [350, 350])
    robot.update_sensor_readings([(0, 0), (100, 100), (700, 400)])

    for i in range(3):
        temp = []
        for j in range(2):
            temp.append((np.random.randint(350-50, 350+50), 250))
        robot.update_sensor_readings(temp)
        # print(len(robot.sensor_readings[0]), robot.sensor_readings[:, -1])
    
    # print(robot.sensor_readings)
    # print(robot._calc_all_dist())
    print(robot._calc_obstacle_repulsion())
    print(np.sum(robot._calc_obstacle_repulsion(), axis=0))
    # print(robot._calc_all_dist())
    # print(robot._calc_goal_attraction())
    # print(robot._calc_wall_x_repulsion())
    # print(0.75*robot._calc_goal_attraction() + 0.25*robot._calc_wall_x_repulsion())
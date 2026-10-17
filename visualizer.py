import matplotlib.pyplot as plt
from matplotlib import patches
from numpy import pi

class Visualizer:
    '''Contains functions required for drawing the map, obstacles, sensor readings and the robot'''

    def __init__(self, x_lim: tuple[float, float], y_lim: tuple[float, float], tolerance_radii: tuple[int, int], obstacles, goal_pos: list[float], robot_pos: list[float] ):
        self.x_min, self.x_max = x_lim
        self.y_min, self.y_max = y_lim
        self.X = x_lim[1] - x_lim[0] # width (x) of map
        self.Y = y_lim[1] - y_lim[0] # 'height' (y) of map
        self.obstacles = obstacles
        self.sensor_readings = []
        self.robot_pos = robot_pos
        self.robot_pos_list = [robot_pos]
        self.goal_pos = goal_pos
        self.step_arrow_patches = []
        self.avoid_radius, self.goal_radius = tolerance_radii
        self._initialize_drawer()

    def _initialize_drawer(self):
        '''Sets axis limits and hides axes'''
        plt.ion()
        plt.show()

        # Set axes and hide them
        self.fig, self.ax = plt.subplots(1, 1)
        self.ax.set_xlim(self.x_min, self.x_max)
        self.ax.set_ylim(self.y_min, self.y_max)
        plt.axis('off')

        # Draw borders
        self.border = patches.Rectangle((self.x_min, self.y_min), self.X, self.Y, edgecolor='blue', facecolor='none', linewidth=5)
        self.ax.add_patch(self.border)

        # Draw obstacles
        self.obstacle_rects = []
        for obst in self.obstacles:
            t = (obst[0][0]-obst[1]/2, obst[0][1]-obst[2]/2)
            rect = patches.Rectangle(t, obst[1], obst[2], facecolor='red', edgecolor='black')
            self.obstacle_rects.append(rect)
            self.ax.add_patch(rect)

        # Draw goal position
        goal_point_radius = 10
        t = (self.goal_pos[0], self.goal_pos[1])
        goal_hexagon = patches.RegularPolygon(t, 6, radius=goal_point_radius, color='green')
        self.ax.add_patch(goal_hexagon)
        self.ax.annotate("Goal", (goal_point_radius, goal_point_radius), xycoords=goal_hexagon, ha='center', va='bottom')

        # Draw goal radius
        t = (self.goal_pos[0], self.goal_pos[1])
        self.goal_circle = patches.Circle(t, self.goal_radius, facecolor='none', linestyle='--', ec='green', linewidth=1)
        self.ax.add_patch(self.goal_circle)

        # Draw robot
        t = (self.robot_pos[0]-6, self.robot_pos[1]-6)
        self.robot_rect = patches.Rectangle(t, 12, 12, angle=45, rotation_point='center', facecolor='orange', ec='black')
        self.ax.add_patch(self.robot_rect)

        # Draw avoidance radius
        t = (self.robot_pos[0], self.robot_pos[1])
        self.avoidance_circle = patches.Circle(t, self.avoid_radius, facecolor='none', linestyle='--', ec='blue', linewidth=1)
        self.ax.add_patch(self.avoidance_circle)

    def update_robot_pos(self, new_robot_pos):
        arrow_patch = patches.Arrow(self.robot_pos[0], self.robot_pos[1],
                                    new_robot_pos[0]-self.robot_pos[0], new_robot_pos[1]-self.robot_pos[1],
                                    width=6, color='blue')
        
        self.ax.add_patch(arrow_patch)
        self.step_arrow_patches.append(arrow_patch)

        self.robot_pos_list.append(new_robot_pos)
        self.robot_pos = new_robot_pos
        t = (self.robot_pos[0]-6, self.robot_pos[1]-6)
        self.robot_rect.set_xy(t)

        t = (self.robot_pos[0], self.robot_pos[1])
        self.avoidance_circle.set_center(t)

    def update_sensor_readings(self, new_readings: list[tuple[float, float]]):
        for reading in new_readings:
            self.sensor_readings.append(reading)
            self.ax.plot(reading[0], reading[1], 'bo')

    def finish(self):
        plt.ioff()
        plt.show()

if __name__ == "__main__":
    drawer = Visualizer((0, 750), (0, 450), (300, 100), [((350, 250), 100, 100)], [700, 400], [100, 100])

    drawer.finish()

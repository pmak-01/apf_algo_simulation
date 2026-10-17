import numpy as np
from visualizer import Visualizer
from robot import Robot
from matplotlib.pyplot import pause

x_lim = (0, 750) # map x bounds
y_lim = (0, 450) # map y bounds
X = x_lim[1] - x_lim[0] # width (x) of map
Y = y_lim[1] - y_lim[0] # 'height' (y) of map

avoid_radius= 200
'''R_th, threshold radius: only obstacle points within this distance from the robot can influence the displacement'''
goal_radius = 100
'''if the robot is within this radius around the goal, it is considered to have reached the goal position'''

obstacles = [
    ((300, 150), 1, 150),
]
'''A list of obstacles, which are approximated as rectangles, represented as tuples `((x,y), width, height)`
If width=1, the program generates sensor data fully on the obstacle, otherwise generated sensor data is all over the field.
'''

robot_pos = [100., 100.] # location of robot (represented as a point)
goal_pos = [650., 350.]

robot = Robot(x_lim, y_lim, (avoid_radius, goal_radius), goal_pos, robot_pos)
viz = Visualizer(x_lim, y_lim, (avoid_radius, goal_radius), obstacles, goal_pos, robot_pos)

t = 0
while not robot.is_at_goal():
    temp = []
    for i in range(5):
        if np.random.rand() <= 0.7 and len(obstacles):
            for obstacle in obstacles:
                if obstacle[1] == 1:
                    temp.append((obstacle[0][0], np.random.randint(obstacle[0][1]-int(obstacle[2]/2), obstacle[0][1]+int(obstacle[2]/2))))
                else:
                    temp.append((np.random.randint(obstacle[0][0]-int(obstacle[2]/2), obstacle[0][0]+int(obstacle[2]/2)), obstacle[0][1]))
        else:
            temp.append((np.random.randint(0, 750), np.random.randint(0, 450)))

    robot.update_sensor_readings(temp)
    robot.calculate_next_move()

    viz.update_sensor_readings(temp)
    viz.update_robot_pos(robot.get_robot_pos())

    pause(0.5)
    t += 1

viz.finish()
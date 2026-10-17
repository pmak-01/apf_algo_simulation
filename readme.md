# Robot Navigation based on the APF algorithm (with simulation)

This project presents a robot navigation algorithm developed based on the Artificial Potential Field (APF) algorithm. Given a point cloud of obstacle locations, the algorithm finds the safest next step towards a goal. This displacement is calculated after every step, so online point cloud updates can be taken into account on the fly.

The mathematics and logic behind the code is explained in [Explanation of Modified APF algorithm.pdf](Explanation%20of%20Modified%20APF%20algorithm.pdf). This project is intended to work in conjuction with another project I made, which uses an ultrasonic distance sensor (HC-SR04) to create a LiDAR-like point cloud ([link to GitHub repo](https://github.com/pmak-01/mapper)).

`robot.py`: Contains all the mathematics to produce the next displacement step. The `step_size` variable must be set by the user, which denotes the distance (in mm) that the robot travels in each step.

`simulator.py`: The program that can be run. The constants in lines 6-19 must be set by the user. To mimic sensor data, this program uses the `np.random` library. If width of obstacle = 1, the program generates sensor data fully on the obstacle, otherwise generated sensor data is all over the field. In the former case, a few random points all over the field are still generated to mimic sensor noise.

`visualizer.py`: Draws the simulation - the robot, boundaries, step vectors, goal, obstacles.
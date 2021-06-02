README for ME 301 Intro to Robotics Lab Assignment 3

The objective: Optimize the hexapod's wall-following walking gait speed by using Q-Learning. The hexapod will learn to adjust its step size based on its distance from the wall. 

In earlier assignments, a walking and turning gait was developed for the hexapod robot. A PID controller was implemented to ensure that the robot walked at a fixed distance from a wall if one was sensed.
In the original walking gait, the robot walks at a fixed step size. Upon implementing Q-Learning, the hexapod will adjust its step size based on how far it is from the wall and how much the controller kicks in to adjust its position.
There are three files provided:
1. asn3.py (This is the python script that implements training, implementation and evaluation)
2. 10x10qmatrix.csv (This is the .csv file that stores the 10x10 q matrix built from the Q-Learning method, it is important that it is in the ros workspace directory)
3. HexapodRobot_asn3.ttt (This is the scene file to be loaded into CoppeliaSim)

Here is a brief explanation of what happens in the asn3.py file:
1. a) If this is the first time that a Q-matrix is being built, the comment 'Q_mat = np.zeros((N, N))' under HexapodControl class must be uncommented, and 'Q_mat = np.genfromtxt("/home/sarah/Documents/ME301/me_cs301_coppeliasim_robots/10x10qmatrix.csv", delimiter=",")' must be commented out. This will start the matrix with 0's. 
1. b) If this is being built off of a previously trained Q-matrix, the comment 'Q_mat = np.zeros((N, N))' under HexapodControl class must be commented and 'Q_mat = np.genfromtxt("/home/sarah/Documents/ME301/me_cs301_coppeliasim_robots/10x10qmatrix.csv", delimiter=",")' must be uncommented. This will load in the .csv file and build off of values already present.
2. Run the python script using the command "rosrun me_cs301_grp asn3.py"
3. Three options will be given: training, implementing and evaluating. 
4. a) 'training' will have the hexapod walk along a wall until the front sensor detects an obstacle. Once an obstacle is detected in front, it will reset the robot's position at (0, 0) and repeat indefinitely.
4. b) 'implementing' will have the hexapod walk through the obstacle course using the Q-matrix to determine its actions at each state that it enters.
4. c) 'evaluating' will have the hexapod walk a distance of 1.5m using the Q-matrix that it built. After each run, it will record the distance traveled, travel time and velocity (distance/time). It will reset its position and indefinitely run trials.

If the user has chosen implementing or evaluating, there are no further steps to take and the hexapod will run in CoppeliaSim. If the user has chosen training, there are some things to note:
5. a) 'random' will have the model randomly select an action at each state in order to populate the Q-table.
5. b) 'epsilon greedy' will have the model implement Epsilon Greedy policy at each state in order to populate the Q-table.

Now the robot will run whichever option is chosen!
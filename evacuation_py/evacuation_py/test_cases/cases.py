from math import pi


class TestCase:
    """ Provide some test cases for a 10x10 map. """

    def __init__(self,initial_pose, gate_position, obstacles):

        self.start_pos0 = [initial_pose[0][0], initial_pose[0][1], initial_pose[0][2]]
        self.start_pos1 = [initial_pose[1][0], initial_pose[1][1], initial_pose[1][2]]
        self.start_pos2 = [initial_pose[2][0], initial_pose[2][1], initial_pose[2][2]]
        self.end_pos = [gate_position[0], gate_position[1], -pi/2]

        num = len(obstacles)

        self.obs = []
        for n in range(num):
            self.obs.append([obstacles[n][0][0],obstacles[n][0][1],obstacles[n][2][0]-obstacles[n][0][0],obstacles[n][1][1]-obstacles[n][0][1]])


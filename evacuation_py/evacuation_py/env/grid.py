import numpy as np
from math import sqrt
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from ..test_cases.cases import TestCase
from ..env.environment import Environment


class Grid:
    """ Grid configuration. """

    def __init__(self, env, map_border, cell_size=0.25):

        self.env = env
        self.cell_size = cell_size
        self.map_border = map_border

        self.n = int(self.env.lx / self.cell_size)
        self.m = int(self.env.ly / self.cell_size)

        self.cell_dia = sqrt(2*self.cell_size**2)

        self.get_obstacle_occupancy()
        self.get_boundary_occupancy()
    
    def get_obstacle_occupancy(self):
        """ Fill grid with obstacles. """

        self.grid = [[0] * self.m for _ in range(self.n)]

        for ob in self.env.obs:
            x1, y1 = self.to_cell_id([ob.x, ob.y])
            x2, y2 = self.to_cell_id([ob.x+ob.w, ob.y+ob.h])

            if (ob.x+ob.w) % self.cell_size == 0:
                x2 -= 1
            
            if (ob.y+ob.h) % self.cell_size == 0:
                y2 -= 1

            for i in range(x1, x2+1):
                for j in range(y1, y2+1):
                    self.grid[i][j] = 1
    
    def to_cell_id(self, pt):
        """" Convert point into grid index. """

        x = min(int(pt[0] / self.cell_size), self.n-1)
        y = min(int(pt[1] / self.cell_size), self.m-1)

        return [x, y]

    def get_boundary_occupancy(self):
        """ Fill grid with boundary occupancy. """

        hexagon_points = []
        for point in self.map_border:
            x, y = self.to_cell_id(point)
            # self.grid[x][y] = 1
            hexagon_points.append([x,y])
        for i in range(len(self.grid)):
            for j in range(len(self.grid)):
                if not is_inside_hexagon([i, j],hexagon_points):
                    self.grid[i][j] = 1
    
    def get_neighbors(self, cell_id):
        """ Get all the 4 adjacent cells. """

        x, y = cell_id
        nbs = []

        for p in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            if 0 <= x + p[0] < self.n and 0 <= y + p[1] < self.m:
                if self.grid[x + p[0]][y + p[1]] == 0:
                    nbs.append([x + p[0], y + p[1]])

        return nbs


def is_inside_hexagon(point,hexagon_points):
    x, y = point
    x_coords = [p[0] for p in hexagon_points]
    y_coords = [p[1] for p in hexagon_points]
    min_x = min(x_coords)
    max_x = max(x_coords)
    min_y = min(y_coords)
    max_y = max(y_coords)

    if x < min_x or x > max_x or y < min_y or y > max_y:
        return False

    return ((x - hexagon_points[0][0]) * (hexagon_points[1][1] - hexagon_points[0][1]) -
            (y - hexagon_points[0][1]) * (hexagon_points[1][0] - hexagon_points[0][0])) >= 0 and \
           ((x - hexagon_points[1][0]) * (hexagon_points[2][1] - hexagon_points[1][1]) -
            (y - hexagon_points[1][1]) * (hexagon_points[2][0] - hexagon_points[1][0])) >= 0 and \
           ((x - hexagon_points[2][0]) * (hexagon_points[3][1] - hexagon_points[2][1]) -
            (y - hexagon_points[2][1]) * (hexagon_points[3][0] - hexagon_points[2][0])) >= 0 and \
           ((x - hexagon_points[3][0]) * (hexagon_points[4][1] - hexagon_points[3][1]) -
            (y - hexagon_points[3][1]) * (hexagon_points[4][0] - hexagon_points[3][0])) >= 0 and \
           ((x - hexagon_points[4][0]) * (hexagon_points[5][1] - hexagon_points[4][1]) -
            (y - hexagon_points[4][1]) * (hexagon_points[5][0] - hexagon_points[4][0])) >= 0 and \
           ((x - hexagon_points[5][0]) * (hexagon_points[0][1] - hexagon_points[5][1]) -
            (y - hexagon_points[5][1]) * (hexagon_points[0][0] - hexagon_points[5][0])) >= 0

def main():

    tc = TestCase()
    env = Environment(tc.obs3)
    grid = Grid(env)

    fig, ax = plt.subplots(figsize=(6,6))
    ax.set_xlim(0, env.lx)
    ax.set_ylim(0, env.ly)
    ax.set_aspect("equal")

    ax.set_xticks(np.arange(0, env.lx, grid.cell_size))
    ax.set_yticks(np.arange(0, env.ly, grid.cell_size))
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.tick_params(length=0)
    plt.grid()
    
    for ob in env.obs:
        ax.add_patch(Rectangle((ob.x, ob.y), ob.w, ob.h, fc='gray', ec='k'))

    plt.show()


if __name__ == '__main__':
    main()
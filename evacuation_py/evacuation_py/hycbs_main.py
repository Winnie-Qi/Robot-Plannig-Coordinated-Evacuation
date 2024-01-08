from copy import deepcopy
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
from matplotlib.collections import PatchCollection
from .env.grid import Grid
from .env.car import SimpleCar
from .env.environment import Environment
from .test_cases.cases import TestCase

from .methods.cbs import *


def main(initial_pose, map_border, gate_position, obstacles):
    print("enter main")    
    print("initial_pose",initial_pose,"map_border",map_border,"gate_position",gate_position,"obstacles",obstacles)
    # 代码里地图的范围是0到16，所以所有xy坐标加8.
    map_border = [[x + 8 for x in sublist] for sublist in map_border]
    gate_position = [x + 8 for x in gate_position]
    obstacles = [[[x + 8 for x in sublist2] for sublist2 in sublist1] for sublist1 in obstacles]
    initial_pose = [[x + 8 if index < 2 else x for index, x in enumerate(sublist)] for sublist in initial_pose]

    tc = TestCase(initial_pose, gate_position, obstacles)

    env = Environment(tc.obs,map_border)

    cars = []
    car0 = SimpleCar(env, tc.start_pos0, tc.end_pos)
    cars.append(car0)
    car1 = SimpleCar(env, tc.start_pos1, tc.end_pos)
    cars.append(car1)
    car2 = SimpleCar(env, tc.start_pos2, tc.end_pos)
    cars.append(car2)

    grid = Grid(env,map_border)

    high_planner = CBS(cars,grid) # CBS层面初始的空限制{}

    start = HighLevelNode() # 高层节点层面初始的空限制{}

    for i in range(len(cars)):
        start.constraint_dict[i] = Constraints() # 用空的Constraints()对象填满高层节点层面的字典

    start.solution,start.dict_path,start.dict_closed_ = high_planner.compute_solution() # 第一次Hybrid A*的结果

    if not start.solution:
        print("no solution")
        return {}

    start.cost = max(len(i) for i in start.solution.values())

    high_planner.open_set |= {start}

    while high_planner.open_set:
        P = min(high_planner.open_set)
        high_planner.open_set -= {P}
        high_planner.closed_set |= {P}
        high_planner.constraint_dict = P.constraint_dict # 高层节点的constraint_dict赋值给了CBS层面的constraint_dict
        conflict_dict = high_planner.get_first_conflict(P.solution) # 两个智能体的冲突信息
        if not conflict_dict:
            print("solution found")
            # P.solution字典
            # P.dict_path字典
            # P.dict_closed_字典
            print(P.solution)
            return P.solution
        print('solving conflict...')
        constraint_dict = high_planner.create_constraints_from_conflict(conflict_dict) # 将冲突转为限制，为两个智能体的
        for agent in constraint_dict.keys():
            P.dict_path = {} # 无法被深拷贝所以置空
            P.dict_closed_ = {}
            new_node = deepcopy(P) #deepcopy了open_set里目前代价最小的高层节点
            new_node.constraint_dict[agent].add_constraint(constraint_dict[agent]) # 将两个智能体的限制集合进高层节点全部限制字典里
            high_planner.constraint_dict = new_node.constraint_dict # 高层节点的constraint_dict赋值给CBS层面的constraint_dict
            new_node.solution,new_node.dict_path,new_node.dict_closed_ = high_planner.compute_solution()
            if not new_node.solution:
                continue
            new_node.cost = max(len(i) for i in new_node.solution.values())
            if new_node not in high_planner.closed_set:
                high_planner.open_set |= {new_node}
    pass
    return None


if __name__ == '__main__':

    map_border = [[-4,6.9],[4,6.9],[8,0],[4,-6.9],[-4,-6.9],[-8,0]]
    gate_position = [0,-6.5]
    obstacles = [[[2.6,0.8],[2.6,1.4],[3.2,1.4],[3.2,0.8]],[[-0.3,-1.5],[-0.3,-2],
                                                            [0.5,-2],[0.5,-1.5]],[[-5.6,2],[-5.6,2.6],[-5,2.6],[-5,2]],[[0.4,-6.5],
                                                                                                                        [0.4,-5.5],[1.3,-5.5],[1.3,-6.5]],[[0.9,-4.5],[0.9,-3.7],[1.6,-3.7],[1.6,-4.5]]]
    initial_pose = [[0,0,0],[2,0,0],[4,0,0]]

    solution, dict_path, dict_closed_ = main(initial_pose, map_border, gate_position, obstacles)

    map_border = [[x + 8 for x in sublist] for sublist in map_border]
    gate_position = [x + 8 for x in gate_position]
    obstacles = [[[x + 8 for x in sublist2] for sublist2 in sublist1] for sublist1 in obstacles]
    initial_pose = [[x + 8 if index < 2 else x for index, x in enumerate(sublist)] for sublist in initial_pose]

    for key in dict_path:
        dict_path[key] = dict_path[key][::5] + [dict_path[key][-1]]

    fig, ax = plt.subplots(figsize=(6,6))
    border_polygon = Polygon(map_border, closed=True, edgecolor='r', facecolor='none')
    ax.add_patch(border_polygon)

    ax.set_xlim(0, 16)
    ax.set_ylim(0, 16)
    ax.set_aspect("equal")

    ax.set_xticks(np.arange(0, 16, 0.25))
    ax.set_yticks(np.arange(0, 16, 0.25))
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.tick_params(length=0)
    plt.grid(which='both')

    for ob in obstacles:
        polygon = Polygon(ob, closed=True, alpha=0.5, facecolor='grey')
        ax.add_patch(polygon)
    polygon = Polygon([[7.5,2],[8.5,2],[8.5,1],[7.5,1]], closed=True, alpha=0.5, facecolor='red')
    ax.add_patch(polygon)

    plt.show()


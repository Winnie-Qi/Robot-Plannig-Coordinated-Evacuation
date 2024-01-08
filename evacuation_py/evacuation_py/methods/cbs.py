from itertools import combinations
import numpy as np

from ..methods.hybrid_astar import HybridAstar
inflation = 0.01

class HighLevelNode(object):
    def __init__(self):
        self.solution = {}
        self.constraint_dict = {}
        self.dict_path = {}
        self.dict_closed_ = {}
        self.cost = 0

    def __eq__(self, other):
        if not isinstance(other, type(self)): return NotImplemented
        return self.solution == other.solution and self.cost == other.cost

    def __hash__(self):
        return hash((self.cost))

    def __lt__(self, other):
        return self.cost < other.cost

class Constraints(object):
    def __init__(self):
        self.vertex_constraints = set()


    def add_constraint(self, other):
        self.vertex_constraints |= other.vertex_constraints

    def __str__(self):
        return "VC: " + str([str(vc) for vc in self.vertex_constraints])

class VertexConstraint(object):
    def __init__(self, time, location):
        self.time = time
        self.location = location

    def __eq__(self, other):
        return self.time == other.time and self.location == other.location

    def __hash__(self):
        return hash(str(self.time) + str(self.location))

    def __str__(self):
        return '(' + str(self.time) + ', ' + str(self.location) + ')'

class Conflict(object):
    VERTEX = 1
    EDGE = 2

    def __init__(self):
        self.time = -1
        self.type = -1

        self.agent_1 = ''
        self.agent_2 = ''

        self.location_1 = Location()
        self.location_2 = Location()

    def __str__(self):
        return '(' + str(self.time) + ', ' + self.agent_1 + ', ' + self.agent_2 + \
               ', ' + str(self.location_1) + ', ' + str(self.location_2) + ')'

class Location(object):
    def __init__(self, x=-1, y=-1):
        self.x = x
        self.y = y

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __str__(self):
        return str((self.x, self.y))

class CBS(object):

    def __init__(self,cars,grid,reverse=False):
        self.cars = cars
        self.distance_threshhold = self.cars[0].carl
        # self.distance_threshhold = 1.5
        self.grid = grid
        self.open_set = set()
        self.closed_set = set()
        self.agent_dict = {}
        self.constraints = Constraints()
        self.constraint_dict = {}
        self.inflation = inflation

    def compute_solution(self):
        solution = {}
        dict_path ={}
        dict_closed_ = {}
        for index,car in enumerate(self.cars):
            self.constraints = self.constraint_dict.setdefault(index, Constraints())
            self.hastar = HybridAstar(index,car,self.grid, reverse=0,constraints=self.constraint_dict)
            path,path_points,closed_ = self.hastar.search_path(heu=True, extra=True) # 单个agent的Hybrid A*结果
            if not path_points:
                print("单个agent的Hybrid A*没路")
                return False
            solution.update({index:(path_points)})
            dict_path.update({index:(path)})
            dict_closed_.update({index:(closed_)})
        return solution,dict_path,dict_closed_

    def get_first_conflict(self, solution):
        max_t = max(len(i) for i in solution.values())
        result = Conflict()
        for t in range(max_t):
            for agent_1, agent_2 in combinations(solution.keys(), 2):
                if t < len(solution[agent_1]):
                    state_1 = solution[agent_1][t]
                else:
                    continue
                if t < len(solution[agent_2]):
                    state_2 = solution[agent_2][t]
                else:
                    continue
                if self.is_equal_except_time(state_1,state_2):
                    result.time = t
                    result.type = Conflict.VERTEX
                    result.location_1 = state_1
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    return result
        return False

    def is_equal_except_time(self,state1,state2):
        dist = np.linalg.norm(
            np.array([state1[0], state1[1]]) - np.array([state2[0], state2[1]]))
        return dist <= self.distance_threshhold

    def create_constraints_from_conflict(self, conflict):
        constraint_dict = {}
        v_constraint = VertexConstraint(conflict.time, conflict.location_1)
        constraint = Constraints()
        constraint.vertex_constraints |= {v_constraint}
        constraint_dict[conflict.agent_1] = constraint
        constraint_dict[conflict.agent_2] = constraint
        return constraint_dict
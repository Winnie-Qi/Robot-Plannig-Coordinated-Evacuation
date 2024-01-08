from ..env.obstacle import Obstacle
from ..utils.intersection import rectangle_ringsector_intersected


class Environment:
    """ The map configuration. """

    def __init__(self, obs, map_border, lx=16, ly=16):

        self.lx  = float(lx)
        self.ly  = float(ly)
        self.map_border = map_border

        if obs:
            self.obs = [Obstacle(*ob) for ob in obs]
        else:
            self.obs = []
    
    def rectangle_inbounds(self, rect, safe_dis=0.05):
        """ Check rectangle target within the map bounds. """

        for v in rect:
            if not self.is_point_inside_hexagon(v):
                return False
            # if v[0] < safe_dis:
            #     return False
            # if v[0] > self.lx - safe_dis:
            #     return False
            # if v[1] < safe_dis:
            #     return False
            # if v[1] > self.ly - safe_dis:
            #     return False
        
        return True

    def is_point_inside_hexagon(self, point):
        x, y = point
        count = 0
        for i in range(len(self.map_border)):
            p1 = self.map_border[i]
            p2 = self.map_border[(i + 1) % len(self.map_border)]
            if ((p1[1] > y) != (p2[1] > y)) and (x < (p2[0] - p1[0]) * (y - p1[1]) / (p2[1] - p1[1]) + p1[0]):
                count += 1

        return count % 2 == 1
    
    def ringsector_inbounds(self, rs, safe_dis=0.05):
        """ Check ringsector target within the map bounds. """

        rect = [[0+safe_dis,        0+safe_dis],
                [self.lx-safe_dis,  0+safe_dis],
                [self.lx-safe_dis,  self.ly-safe_dis],
                [0+safe_dis,        self.ly-safe_dis]]
        
        return not rectangle_ringsector_intersected(rect, rs, False)

    def rectangle_obstacle_free(self, rect):
        """ Check rectangle target is obstacle-free or not. """

        for ob in self.obs:
            if not ob.rectangle_safe(rect):
                return False
        
        return True
    
    def ringsector_obstacle_free(self, rs):
        """ Check ringsector target is obstacle-free or not. """

        for ob in self.obs:
            if not ob.ringsector_safe(rs):
                return False
        
        return True

    def rectangle_safe(self, rect):
        """ Check rectangle target is safe or not. """

        if self.rectangle_inbounds(rect) and self.rectangle_obstacle_free(rect):
            return True
        
        return False
    
    def ringsector_safe(self, rs):
        """ Check ringsector target is safe or not. """

        if self.ringsector_inbounds(rs) and self.ringsector_obstacle_free(rs):
            return True
        
        return False

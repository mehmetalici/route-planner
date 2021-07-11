import numpy as np
import copy

class _Node:
    def __init__(self, id, x, y, is_goal=False):
        self.is_explored = False
        self.is_goal = is_goal
        self.id = id
        self.x = x
        self.y = y
        
    def __sub__(self, other):
        return np.linalg.norm([self.x, self.y], [other.x, other.y])

    
class _Path:
    def __init__(self, starting_node=None):
        self.nodes = []
        if starting_node is not None:
            self.nodes.append(starting_node)
        self.cost = 0
        self.is_blocked = False
        self.is_reached = False

    def _append(self, new):
        return self.nodes.append(new)

    def peek(self):
        return self.nodes[-1]

    def add_intersection(self, intersection, cost, in_place=True):
        path = self if in_place else self.copy()

        path._append(intersection)
        if cost == 0:
            path.is_reached = True
        path.cost += cost

        return None if in_place else path

    def copy(self):
        return copy.copy(self)
    

class _ShortestPath:
    def __init__(self, m):
        self.map = m
        self.nodes = []
        self.roads = []
        self.paths = []

    def _populate_nodes_by_map(self):
        self.nodes = [None] * len(self.map.intersections)
        for idx, point in self.map.intersections.items():
            self.nodes[idx] = _Node(id=idx, x=point[0], y=point[1])

    def _populate_roads_by_map(self):
        # Init roads
        self.roads = []
        for adjacent_node_indices in self.map.roads:
            self.roads.append([self.nodes[adjacent_node_idx] for adjacent_node_idx in adjacent_node_indices])

    def _expand(self, intersection: int) -> list:
        new_frontiers = []
        for adjacent_node in self.roads[intersection]:
            if not adjacent_node.is_explored or adjacent_node.is_goal:
                adjacent_node.is_explored = True
                new_frontiers.append(adjacent_node.id)            
        return new_frontiers
    
    def _calculate_step_cost(self, current_isec: int, next_isec: int, goal: int):
        g = self.distance(current_isec, next_isec)
        h = self.distance(next_isec, goal)
        f = g + h
        return f

    def _get_path_by_last_intersection(self, isec_current: int):               
        for path in self.paths:
            if path.peek() == isec_current:
                return path
        raise PathNotFoundError

    def _get_min_cost_path(self):
        return min(self.paths, key=lambda path: path.cost)


    def shortest_path(self, start: int, goal: int):
        # Clean-up for the current session
        self._populate_nodes_by_map()
        self._populate_roads_by_map()
        self.nodes[goal].is_goal = True

        # Init
        frontiers = []
        
        new_frontiers = self._expand(start)
        frontiers.append(new_frontiers)
        self.paths.append(_Path(start))
        
        # Main Loop
        current_intersection = start
        while len(frontiers) != 0:
            new_frontiers = self._expand(current_intersection)
            path_to_frontiers = self._get_path_by_last_intersection(current_intersection)
            for frontier in new_frontiers:
                step_cost = self.calculate_step_cost(current_intersection, frontier, goal)
                path_with_frontier = path_to_frontiers.add_intersection(frontier, step_cost, in_place=False)
                self.paths.append(path_with_frontier)              
            
            current_intersection = self._get_min_cost_path().peek()
            frontiers.remove(current_intersection)
            self.paths.remove(path_to_frontiers)
            frontiers.append(new_frontiers)

        reached_paths = filter(lambda path: path.is_reached, self.paths)
        return min(reached_paths, key=lambda path: path.cost)


def distance(self, isec_1: int, isec_2: int):
    return self.nodes[isec_1] - self.nodes[isec_2]
    

class PathNotFoundError(Exception):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)


def shortest_path(m, start: int, end: int):
    sp = _ShortestPath(m)
    return sp.shortest_path(start, end)
    
    
    
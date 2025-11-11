import networkx as nx
from typing import Tuple, List, Dict
from map_module.map import Map
from path_planner.path_planner import PathPlanner

class AstarPlanner(PathPlanner):
    def __init__(self, map_input: Map):
        super().__init__(map_input)
        self.__G = map_input.G

    def plan_path(self, start: int, goal: int) -> List[int]:
        path = []
        try:
            path = nx.shortest_path(self.__G, start, goal)
        except nx.NetworkXNoPath:
            print('Warning: No Path Found!')
        return path

    def plan_paths(self, starts: Dict[int, int], goals: Dict[int, int]) \
            -> Dict[int, List[int]]:
        robot_paths = dict()
        if len(starts) != len(goals):
            print('Warning: Starts and Goals do not match!')
            return robot_paths

        for robot_id in starts:
            start_id = starts[robot_id]
            goal_id = goals[robot_id]
            path = self.plan_path(start_id, goal_id)
            robot_paths[robot_id] = path

        return robot_paths
    
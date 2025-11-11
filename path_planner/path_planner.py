from abc import abstractmethod

from typing import Tuple, Dict, List
from map_module.map import Map


class PathPlanner:
    def __init__(self, map_input: Map):
        self._map = map_input
        self._nodes = map_input.nodes()
        self._node_pos2id = {node['pos']: node['id'] for node in self._nodes.values()}

    def node_pos2id(self, pos: Tuple[int, int]):
        return self._node_pos2id.get(pos, None)

    @abstractmethod
    def plan_path(self, start: int, goal: int) -> List[int]:
        pass

    @abstractmethod
    def plan_paths(self, starts: Dict[int, int], goals: Dict[int, int]) \
            -> Dict[int, List[int]]:
        pass

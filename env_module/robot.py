from typing import List

from map_module.map import Map


class Robot:

    class RobotStatus:
        """
        机器人状态
        IDLE = "IDLE"                              # 空闲
        BUSY = "BUSY"                              # 運行
        DELIVERY = "DELIVERY"                      # 運輸
        """
        IDLE = "IDLE"
        BUSY = "BUSY"
        DELIVERY = "DELIVERY"

    def __init__(self, robot_id: int, node_id: int, goal_id: int, map_input: Map):
        self.__id = robot_id
        self.__node_id = node_id
        self.__goal_id = goal_id
        self.__dir = 0
        self.__path = []
        self.__map = map_input
        self.__state = self.RobotStatus.IDLE

    def id(self):
        return self.__id

    def node_id(self) -> int:
        return self.__node_id

    def goal_id(self) -> int:
        return self.__goal_id

    def set_goal_id(self, goal_id: int):
        self.__goal_id = goal_id

    def pos(self):
        return self.__map.id2pos(self.__node_id)

    def state(self) -> str:
        return self.__state

    def set_state(self, state: str):
        self.__state = state

    def next_node_id(self):
        if len(self.__path) > 0:
            return self.__path[0]
        else:
            return self.__node_id

    def dir(self):
        if len(self.__path) == 0:
            return self.__dir
        if len(self.__path) >= 1 and self.__path[0] == self.id():
            return self.__dir
        next_pos = self.__map.id2pos(self.__path[0])
        if next_pos[0] == self.pos()[0]:
            if next_pos[1] > self.pos()[1]:
                return 1
            else:
                return 3
        else:
            if next_pos[0] > self.pos()[0]:
                return 0
            else:
                return 2

    def path(self):
        return self.__path

    def set_path(self, path: List[int]):
        self.__path = path

    def move(self):
        # move to the next node
        if len(self.__path) > 0:
            self.__node_id = self.__path.pop(0)
        # update direction
        self.__dir = self.dir()

    def __str__(self):
        robot_str = 'robot {}: '.format(self.__id)
        robot_str += 'node id: {}, '.format(self.__node_id)
        robot_str += 'goal id: {}, '.format(self.__goal_id)
        # robot_str += 'direction: {}, '.format(self.__dir)
        robot_str += 'path: {};'.format(self.__path)
        return robot_str

    # def state(self):
    #     return self.__state
    # def change_state(self, new_state):
    #     print(f"robot{self.__id} origin state is {self.__state}")
    #     self.__state = new_state
    #     print(f"changed to {new_state} from {self.pos()[0]} to {self.pos()[1]}")

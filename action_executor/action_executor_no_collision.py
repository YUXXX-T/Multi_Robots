from typing import Dict
from env_module.robot import Robot
from action_executor.action_executor import ActionExecutor


class ActionExecutorNoCollision(ActionExecutor):
    def __init__(self, robots: Dict[int, Robot]):
        super().__init__(robots)

    def execute_actions(self):
        reserved_node_ids = dict()
        for robot in self._robots.values():
            reserved_node_ids[robot.node_id()] = robot.id()

        for robot in self._robots.values():
            if robot.next_node_id() in reserved_node_ids and reserved_node_ids[robot.next_node_id()] != robot.id():
                continue

            robot.move()
            reserved_node_ids[robot.node_id()] = robot.id()

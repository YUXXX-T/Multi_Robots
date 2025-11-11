from typing import Dict
from env_module.robot import Robot


class ActionExecutor:
    def __init__(self, robots: Dict[int, Robot]):
        self._robots = robots

    def execute_actions(self):
        for robot in self._robots.values():
            robot.move()

    def __str__(self):
        executor_str = 'The robot states are:\n'
        # executor_str += '\nThe current states of all robots are as follows:\n'
        for robot in self._robots.values():
            executor_str += str(robot) + '\n'
        # executor_str += '\n' + '#' * 100
        return executor_str

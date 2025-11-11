import math
from scipy.optimize import linear_sum_assignment
from typing import List,Dict,Tuple,Optional,Set
import heapq, sys
from dataclasses import field
from enum import Enum
from map_module.map import Map
import numpy as np
from collections import deque


class MultiTask_KM_Allocator:
    """
    核心功能：
    1. 机器人数 > 任务数：部分机器人空闲
    2. 机器人数 < 任务数：部分任务等待下一轮
    3. 机器人数 = 任务数：标准 KM 算法
    """

    def __init__(self, map_path, robots, tasks):
        map_path = 'random-32-32-10.json'
        self.demo_map = Map(map_path)
        self.robots = robots
        self.tasks = tasks
        self.allocation = {}  # {robot_id: task_index}

    def _calculate_distance(self, pos1: int, pos2: int) -> float:
        # 計算歐式距離
        coord1 = self.demo_map.id2pos(pos1)
        coord2 = self.demo_map.id2pos(pos2)
        dx = coord1[0] - coord2[0]
        dy = coord1[1] - coord2[1]
        return math.sqrt(dx*dx+dy*dy)

    def _calculate_task_cost(self, robot_id: int, task_goal: int) -> float:
        """
        计算机器人执行任务的代价
        参数：
            robot_id: 机器人 ID
            task_goal: 任务目标位置
        返回：
            代价（距离）
        """
        robot = self.robots[robot_id]
        # current_pos = robot.start
        current_pos = robot.node_id()

        distance = self._calculate_distance(current_pos, task_goal)
        return distance

    def _build_cost_matrix(self)-> Tuple[np.ndarray, List[int], List[int]]:
        """
        构建代价矩阵
        返回：
            cost_matrix: 代价矩阵
            robot_ids: 机器人 ID 列表（对应矩阵行）
            task_indices: 任务索引列表（对应矩阵列）
        """
        robot_ids = list(self.robots.keys())
        task_indices = list(range(len(self.tasks)))

        n_robots = len(robot_ids)
        n_tasks = len(task_indices)

        size = max(n_robots, n_tasks)
        cost_matrix = np.zeros((size, size))
        for i, robot_id in enumerate(robot_ids):
            for j, task_id in enumerate(task_indices):
                task_goal = self.tasks[task_id]['goal']
                cost = self._calculate_task_cost(robot_id, task_goal)
                cost_matrix[i][j] = cost

        # 虚拟节点代价
        VIRTUAL_COST = sys.maxsize

        # 机器人多于任务：填充虚拟任务
        if n_robots > n_tasks:
            cost_matrix[:n_robots, n_tasks:] = VIRTUAL_COST
        if n_tasks > n_robots:
            cost_matrix[n_robots:, :n_tasks] = VIRTUAL_COST

        return cost_matrix, robot_ids, task_indices

    def allocate(self) -> Dict[int, int]:
        """
        执行 KM 分配算法
        返回：
        {robot_id: task_index} 分配结果字典
        """
        if not self.robots:
            print("There are not existing robots")
            return {}

        if not self.tasks:
            print("There are not existing tasks")
            return {}

        n_robots = len(self.robots)
        n_tasks = len(self.tasks)
        print(f"\n{'=' * 80}")
        print(f"KM 任务分配")
        print(f"{'=' * 80}")
        print(f"机器人数量：{n_robots}")
        print(f"任务数量：{n_tasks}")

        # 1. 构建代价矩阵
        cost_matrix, robot_ids, task_indices = self._build_cost_matrix()
        # 2. 匈牙利算法
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        # 3. 解析
        allocation = {}
        for i, j in zip(row_ind, col_ind):
            is_real_robot = i < n_robots
            is_real_task = j < n_tasks

            if not is_real_robot or not is_real_task:
                # 虛擬分配
                if is_real_robot and not is_real_task:
                    robot_id = robot_ids[i]
                    print(f"  机器人 {robot_id:2d} → 空闲（无任务分配）")
                elif not is_real_robot and is_real_task:
                    task_idx = task_indices[i]
                    print(f"  任务 {task_idx:2d} → 未分配（等待下一轮）")
                continue

            robot_id = robot_ids[i]
            task_idx = task_indices[j]
            task_goal = self.tasks[task_idx]['goal']

            # 記錄分配
            allocation[robot_id] = task_idx
            # 打印信息
            robot = self.robots[robot_id]
            distance = cost_matrix[i][j]
            print(f"     机器人 {robot_id:2d} ← 任务 {task_idx:2d}")
            print(f"     起点：{robot.node_id():4d} → 目标：{task_goal:4d}")
            print(f"     距离：{distance:6.1f}")

        print(f"{'-' * 80}")
        print(f"成功分配：{len(allocation)} 个任务")
        print(f"空闲机器人：{n_robots - len(allocation)} 个")
        print(f"等待任务：{n_tasks - len(allocation)} 个")
        print(f"{'=' * 80}\n")

        self.allocation = allocation
        return allocation



    def get_allocation(self) -> Dict[int, int]:
        return self.allocation

    def get_robot_goal_dict(self) -> Dict[int, int]:
        """
        获取机器人到目标位置的映射（用于路径规划）
        返回：
        {robot_id: goal_position}
        """
        robot_goal_dict = {}
        for robot_id, task_idx in self.allocation.items():
            robot_goal_dict[robot_id] = self.tasks[task_idx]['goal']

        return robot_goal_dict

    def get_robot_start_dict(self) -> Dict[int, int]:
        """
        获取机器人到起始位置的映射
        返回：
        {robot_id: start_position}
        """
        robot_start_dict = {}
        for robot_id, robot in self.robots.items():
            robot_start_dict[robot_id] = robot.start
        return robot_start_dict


class TaskSchedular:
    """
    多轮任务调度器
    功能：
        1. 初始分配任务
        2. 跟踪机器人状态
        3. 机器人完成任务后自动进行下一轮分配
        4. 管理未分配任务队列
    """

    # class RobotStatus:
    #     """
    #     机器人状态
    #     IDLE = "IDLE"                              # 空闲
    #     BUSY = "BUSY"                              # 運行
    #     COMPLETED = "COMPLETED"                    # 完成
    #     """
    #     IDLE = "IDLE"
    #     BUSY = "BUSY"
    #     COMPLETED = "COMPLETED"


    def __init__(self, map_path, robots: Dict, all_tasks: List[Dict]):
        """
        初始化调度器
        参数：
            demo_map: 地图实例
            robots: {robot_id: Robot} 机器人字典
            all_tasks: [{'goal': goal_position}, ...] 所有任务列表
        """
        map_path = 'random-32-32-10.json'
        self.map_path  = map_path
        self.demo_map  = Map(map_path)
        self.robots    = robots
        self.all_tasks = all_tasks

        # 任务队列（未分配的任务）  deque雙端隊列
        self.pending_tasks = deque(range(len(all_tasks)))

        # 已分配任务 {robot_id: task_index}
        self.assigned_tasks: Dict[int, int] = {}
        # 已完成任务 {robot_id: [task_index, ...]}
        self.completed_tasks: Dict[int, List[int]] = {rid: [] for rid in robots.keys()}

        # 機器人狀態
        self.robot_status: Dict[int, str] = {
            # rid: self.RobotStatus.IDLE for rid in robots.keys()
            rid: robot.state() for rid, robot in robots.items()
        }
        # 機器人當前位置
        self.robot_positions: Dict[int, int] = {
            rid: robot.node_id() for rid, robot in robots.items()
        }

        # 分配倫茨
        self.round_number = 0
        # 統計
        self.total_distance = {rid: 0.0 for rid in robots.keys()}

    def _calculate_distances(self, pos1:int, pos2:int) -> float:
        coord1 = self.demo_map.id2pos(pos1)
        coord2 = self.demo_map.id2pos(pos2)
        return math.sqrt((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2)

    def get_idle_robots(self) -> Dict:
        idle_robots = {}
        # for robot_id, status in self.robot_status.items():
        #     # if status == self.RobotStatus.IDLE:
        #     if status == "IDLE":
        #         # 創建臨時Robot對象于分配
        #         class TempRobot:
        #             def __init__(self, robot_id, start):
        #                 self.robot_id = robot_id
        #                 self.start = start
        #         idle_robots[robot_id] = TempRobot(robot_id, self.robot_positions[robot_id])
        for robot_id, robot in self.robots.items():
            # if status == self.RobotStatus.IDLE:
            if robot.state() == "IDLE":
               idle_robots[robot_id] = robot
        return idle_robots

    def get_pending_tasks(self) -> List[Dict]:
        """获取待分配任务"""
        pending_task_list = []
        for task_idx in self.pending_tasks:
            pending_task_list.append(self.all_tasks[task_idx])
        return pending_task_list


    def allocate_round(self, verbose: bool = True) -> Dict[int, int]:
        """
        执行一轮分配
        返回： {robot_id: task_index} 本轮分配结果
        """
        self.round_number += 1
        # 获取空闲机器人和待分配任务
        idle_robots = self.get_idle_robots()
        pending_task_list = list(self.pending_tasks)

        if not idle_robots:
            if verbose:
                print(f"\n 第 {self.round_number} 轮分配：没有空闲机器人")
            return {}

        if not self.pending_tasks:
            if verbose:
                print(f"\n 第 {self.round_number} 轮分配：所有任务已分配")
            return {}
        if verbose:
            print(f"\n{'#' * 80}")
            print(f"第 {self.round_number} 轮任务分配")
            print(f"{'#' * 80}")

        # 執行KM分配
        km = MultiTask_KM_Allocator(self.map_path, idle_robots, [self.all_tasks[idx] for idx in pending_task_list])
        allocation = km.allocate()

        # 更新状态
        for robot_id, relative_task_idx in allocation.items():
            # 正确映射：相对索引 -> 绝对任务ID
            absolute_task_idx = pending_task_list[relative_task_idx]

            robot = self.robots[robot_id]
            robot_start = self.robots[robot_id].node_id()
            task_goal = self.all_tasks[absolute_task_idx]['goal']

            self.assigned_tasks[robot_id] = absolute_task_idx

            robot.set_state("BUSY")
            robot.set_goal_id(task_goal)
            self.robot_status[robot_id] = "BUSY"   # 想想怎麽修改到 把原本傳入的robots的狀態進行更新
            self.pending_tasks.remove(absolute_task_idx)
            print(f"机器人 {robot_id}: 任务 {absolute_task_idx} ({robot_start} -> {task_goal})")
        return allocation


    def complete_task(self, robot_id: int) -> None:
        if robot_id not in self.assigned_tasks:
            print("Error with wrong robot")
            return

        task_idx = self.assigned_tasks[robot_id]
        task_goal = self.all_tasks[task_idx]['goal']

        start_pos = self.robot_positions[robot_id]
        distance = self._calculate_distances(start_pos, task_goal)

        # 更新位置和狀態
        robot = self.robots[robot_id]
        robot.set_state("IDLE")

        self.robot_positions[robot_id] = task_goal
        # self.robot_status[robot_id] = self.RobotStatus.IDLE
        self.robot_status[robot_id] = "IDLE"
        self.completed_tasks[robot_id].append(task_idx)
        self.total_distance[robot_id] += distance

        del self.assigned_tasks[robot_id]

        print(f"\n   机器人 {robot_id} 完成任务 {task_idx}")
        print(f"     {start_pos:4d} → {task_goal:4d}，距离：{distance:6.1f}")
        print(f"     累计距离：{self.total_distance[robot_id]:6.1f}")
        print(f"     已完成任务数：{len(self.completed_tasks[robot_id])}")


    def complete_all_current_tasks(self):
        """完成当前所有忙碌机器人的任务"""
        busy_robots = [
            rid for rid, status in self.robot_status.items()
            # if status == self.RobotStatus.BUSY
            if status == "BUSY"
        ]
        print(f"\n完成本轮 {len(busy_robots)} 个机器人的任务...")

        for robot_id in busy_robots:
            self.complete_task(robot_id)



    def run_full_schedule(self) -> Dict:
        """
        运行完整的多轮调度
        返回： 统计信息字典
        """
        print(f"\n{'=' * 80}")
        print(f"开始多轮任务调度")
        print(f"{'=' * 80}")
        print(f"总机器人数：{len(self.robots)}")
        print(f"总任务数：{len(self.all_tasks)}")
        print(f"{'=' * 80}\n")

        while self.pending_tasks or self.assigned_tasks:
            # 执行一轮分配
            allocation = self.allocate_round()
            # 如果本轮没有分配（所有机器人都忙），等待完成
            if not allocation and self.assigned_tasks:
                print(f"\n 等待机器人完成当前任务...")
            # 完成本轮任务
            self.complete_all_current_tasks()

            # 检查是否还有待分配任务
            if not self.pending_tasks:
                break

            # 打印最终统计
        return self.print_final_statistics()


    def print_final_statistics(self) -> Dict:
        """打印最终统计信息"""
        print(f"\n{'=' * 80}")
        print(f"调度完成统计")
        print(f"{'=' * 80}")

        total_completed = sum(len(tasks) for tasks in self.completed_tasks.values())
        total_dist = sum(self.total_distance.values())

        print(f"\n【总体统计】")
        print(f"  总任务数：{len(self.all_tasks)}")
        print(f"  已完成任务：{total_completed}")
        print(f"  分配轮次：{self.round_number}")
        print(f"  总移动距离：{total_dist:.2f}")
        print(f"  平均每机器人距离：{total_dist / len(self.robots):.2f}")

        print(f"\n【机器人详情】")
        for robot_id in sorted(self.robots.keys()):
            completed = len(self.completed_tasks[robot_id])
            distance = self.total_distance[robot_id]
            print(f"  机器人 {robot_id:2d}：完成 {completed:2d} 个任务，"
                  f"移动距离 {distance:7.2f}")

        print(f"{'=' * 80}\n")

        return {
            'total_tasks': len(self.all_tasks),
            'completed_tasks': total_completed,
            'total_rounds': self.round_number,
            'total_distance': total_dist,
            'robot_details': {
                rid: {
                    'completed': len(self.completed_tasks[rid]),
                    'distance': self.total_distance[rid]
                }
                for rid in self.robots.keys()
            }
        }

    def get_current_allocation(self) -> Tuple[Dict[int, int], Dict[int, int]]:
        """
        获取当前分配的起点和终点字典（用于路径规划）

        返回：
            (robot_start_id_dict, robot_goal_id_dict)
        """
        robot_start_id_dict = {}
        robot_goal_id_dict = {}

        for robot_id, task_idx in self.assigned_tasks.items():
            robot_start_id_dict[robot_id] = self.robot_positions[robot_id]
            robot_goal_id_dict[robot_id] = self.all_tasks[task_idx]['goal']

        return robot_start_id_dict, robot_goal_id_dict

    def get_current_allocation_with_delivery(self) -> Tuple[Dict[int, int], Dict[int, Tuple[int, int]]]:
        """
        获取当前分配的起点和终点字典（用于路径规划）
        返回：
            (robot_start_id_dict, robot_goal_id_dict)
            robot_goal_id_dict: robot_id: ['goal', 'delivery_goal']
        """
        robot_start_id_dict = {}
        robot_goal_id_dict = {}

        for robot_id, task_idx in self.assigned_tasks.items():
            robot_start_id_dict[robot_id] = self.robot_positions[robot_id]
            robot_goal_id_dict[robot_id] = (self.all_tasks[task_idx]['goal'], self.all_tasks[task_idx]['delivery_goal'])

        return robot_start_id_dict, robot_goal_id_dict
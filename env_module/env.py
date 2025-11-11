import os
from tqdm import tqdm
import random
from map_module.map import Map
from env_module.robot import Robot
from env_module.board import Board
from path_planner import AstarPlanner
from path_planner import CAstarPlanner
from action_executor import ActionExecutor
from logger_module.Logger import ConfigurableLogger
from task_assignment.KM import KM
from task_assignment.KM_update import MultiTask_KM_Allocator, TaskSchedular
import time
from colorama import init, Fore, Style
from typing import List, Dict, Tuple, Set, Optional

class Env:
    def __init__(self, map_path, slover='ca_star', seed=0):
        self.demo_map = Map(map_path)
        self.board = Board(self.demo_map, 1000)
        self.seed = seed
        self.solvers = {
            'astar': AstarPlanner,
            'ca_star': CAstarPlanner,
            #'your_Alg': Your/Own/Plannner/Class,
        }
        # =========================== TODO: 实现任意one-shot MAPF算法，实现多机器人无碰完成任务 ===========================
        # self.planner = self.slovers['your_Alg'](self.demo_map)

        # 示例：A* planner
        self.planner = self.solvers[slover](self.demo_map)

        # CA*多亂路徑規劃
        self.global_reservation_table: Dict[Tuple[int, int], int] = {}
        self.global_time_offset: int = 0

    # 随机生成任务点和小车起始点
    def generate_random_tasks(self, num_tasks, seed=0):
        normal_nodes = self.demo_map.nodes_by_type('normal')
        if len(normal_nodes) < num_tasks * 2:
            raise ValueError("Nodes are not enough for unique task and robot starts")
        random.seed(seed)
        # chosen_nodes = random.sample(normal_nodes, num_tasks * 2)
        chosen_nodes = random.sample(normal_nodes, num_tasks * 3)
        # print(chosen_nodes)
        part_size = num_tasks
        robot_start_points = chosen_nodes[:part_size]
        # print(f"len1: {len(robot_start_points)}, {robot_start_points[-1]}")
        task_goal_points = chosen_nodes[part_size:part_size*2]
        # print(f"len2: {len(task_goal_points)}, {task_goal_points[0]} {task_goal_points[-1]}")
        delivery_goal_points = chosen_nodes[part_size*2:]
        # print(f"len3: {len(delivery_goal_points)}, {delivery_goal_points[0]}")
        tasks = []

        for goal in task_goal_points:
            tasks.append({
                'goal': goal,
            })
        robots = {
            i: Robot(i, start, 1, self.demo_map)
            for i, start in enumerate(robot_start_points)
        }
        # for goal, delivery in zip(task_goal_points, delivery_goal_points):
        #     tasks.append({
        #         'goal': goal,
        #         'delivery_goal': delivery
        #     })
        robots = {
            i: Robot(i, start, 1, self.demo_map)
            for i, start in enumerate(robot_start_points)
        }

        return tasks, robots
    
    def calculate_total_path_length(self, paths):
        total_length = 0
        for path in paths.values():
            total_length += len(path) - 1
        return total_length
    
    
    # def generate_task_assignments(self, robots, tasks, robot_start_id_dict, robot_goal_id_dict):
    #
    #     # ============================== TODO: 任务分配类KM实现 ======================================
    #     # km = KM(self.demo_map, robots, tasks)
    #     # =====================================================================================
    #
    #     # row, col = km.compute()
    #     # print("ros: ", row)
    #     # print("col: ", col)
    #     # task_assignments = {}
    #     #
    #     # for robot_id, task_idx in zip(row, col):
    #     #     if task_idx <= len(tasks):
    #     #         task = tasks[task_idx]
    #     #         task_assignments[robots[robot_id].id()] = {
    #     #             'robot_id': robots[robot_id].id(),
    #     #             'task_id': task_idx,
    #     #             'start_id':robots[robot_id].node_id(),
    #     #             'goal_id':task['goal']
    #     #
    #     #         }
    #     # for robot_id, assignment in task_assignments.items():
    #     #     start_node_id = assignment['start_id']
    #     #     goal_node_id = assignment['goal_id']
    #     #     robot_start_id_dict[robot_id] = start_node_id
    #     #     robot_goal_id_dict[robot_id] = goal_node_id
    #     #
    #     # for robot in robots.values():
    #     #     robot_id = robot.id()
    #     #     goal_id = task_assignments.get(robot_id)['start_id']
    #     #     if goal_id:
    #     #         robot.set_goal_id(goal_id)
    #
    #     km = MultiTask_KM_Allocator(self.demo_map, robots,tasks)
    #     allocation = km.allocate()
    #
    #     task_assignments = {}
    #     for robot_id, task_idx in allocation.items():
    #         task = tasks[task_idx]
    #         task_assignments[robot_id] = {
    #             'robot_id': robot_id,
    #             'task_id': task_idx,
    #             'start_id': robots[robot_id].node_id(),
    #             'goal_id': task['goal']
    #         }
    #     for robot_id, assignment in task_assignments.items():
    #         robot_start_id_dict[robot_id] = assignment['start_id']
    #         robot_goal_id_dict[robot_id] = assignment['goal_id']
    #
    #     for robot_id, assignment in task_assignments.items():
    #         goal_id = assignment['goal_id']
    #         robots[robot_id].set_goal_id(goal_id)

    def execute_round(self, robots, robot_paths, executor, logger, fig_save_dir, round_num):
        """执行一轮任务"""
        # 设置路径
        for robot_id in robot_paths:
            robots[robot_id].set_path(robot_paths[robot_id])

        # 初始可视化
        self.board.show_robots(robots, robot_paths,
                               filename=f'{fig_save_dir}/round{round_num}_step0',
                               plt_show=False)
        # 执行直到本轮完成
        timestep = 0
        for timestep in range(10000):
            moving = [r for r in robots.values() if len(r.path()) > 0]
            if not moving:
                break
            logger.info(f'Round {round_num}, Timestep {timestep}: ' + str(executor))
            executor.execute_actions()

            # 可视化
            self.board.show_robots(robots, robot_paths,
                                   filename=f'{fig_save_dir}/round{round_num}_step{timestep + 1}',
                                   plt_show=False)

        print(f"第 {round_num + 1} 轮完成，耗时 {timestep} 步")
        return timestep + 1


    def run(self, log_dir):
        logger = ConfigurableLogger('./logger_module/logging.conf', log_dir)

        fig_save_dir = os.path.join(log_dir, 'figs')
        if not os.path.exists(fig_save_dir):
            os.makedirs(fig_save_dir)
        robot_start_id_dict, robot_goal_id_dict = dict(), dict()
        tasks = list()
        num_tasks =25
        robot_num = 10

        robots = dict()
        tasks, all_robots = self.generate_random_tasks(num_tasks, seed=self.seed)
        robots = {i: all_robots[i] for i in range(robot_num)}

        scheduler = TaskSchedular(self.demo_map, robots, tasks)
        # =============================== 任务分配算法调用, 编写好之后取消注释 ===========================
        # self.generate_task_assignments(robots, tasks, robot_start_id_dict, robot_goal_id_dict)
        # ==============================================================================


        # =============================== 任务分配示例, 编写好KM之后将其注释 ===========================
        # 顺序分配
        # for i, task in enumerate(tasks):
        #     goal_id = task['goal']
        #     # print("goal_id: ", goal_id)
        #     robot_goal_id_dict[i] = goal_id
        #     robot_start_id_dict[i] = robots[i].node_id()
        # ==============================================================================

        executor = ActionExecutor(robots)


        # ========== 多轮调度 =========
        all_rounds_data = []  # 存储每轮数据
        total_timesteps = 0

        while scheduler.pending_tasks or scheduler.assigned_tasks:
            scheduler.allocate_round()
            for rb_id, rb_a in robots.items():
                print("--"*25, "\n", f"robot_id {rb_id}: state {rb_a.state()}, goal_id {rb_a.node_id()}")

            robot_start_id_dict, robot_goal_id_dict = scheduler.get_current_allocation()  # 單點demo
            # robot_start_id_dict, robot_goal_id_dict = scheduler.get_current_allocation_with_delivery()
            if not robot_start_id_dict:
                break

            print(f"\n{'=' * 60}")
            print(f"第 {len(all_rounds_data) + 1} 轮规划开始")
            print(f"当前全局时间偏移: {self.global_time_offset}")
            print(f"当前预定表大小: {len(self.global_reservation_table)}")
            print(f"{'=' * 60}")

            # 传入和接收reservation_table
            robot_paths, updated_reservation_table, new_time_offset = \
                self.planner.plan_paths_with_state(
                    robot_start_id_dict=robot_start_id_dict,
                    robot_goal_id_dict=robot_goal_id_dict,
                    reservation_table=self.global_reservation_table,
                    time_offset=self.global_time_offset,
                    delivery = False
                )

            # 更新全局狀態
            self.global_reservation_table = updated_reservation_table
            self.global_time_offset = new_time_offset

            if not robot_paths:
                print("規划失敗")
                break

            # 路徑規劃
            # robot_paths = self.planner.plan_paths_from_CAstar(robot_start_id_dict, robot_goal_id_dict)
            # 執行本輪任務
            round_steps = self.execute_round(
                robots, robot_paths, executor, logger,
                fig_save_dir, len(all_rounds_data)
            )
            total_timesteps += round_steps
            # 標記已完成
            scheduler.complete_all_current_tasks()

            all_rounds_data.append({
                'paths': robot_paths,
                'assignments': scheduler.assigned_tasks.copy()
            })
            print(f"第 {len(all_rounds_data)} 輪完成，耗時 {round_steps} 步")
            print(f"更新後時間偏移: {self.global_time_offset}")
            print(f"更新後預定表大小: {len(self.global_reservation_table)}\n")

        scheduler.print_final_statistics()


        # 路径规划planner调用
        # robot_paths = self.planner.plan_paths_from_CAstar(robot_start_id_dict, robot_goal_id_dict)
        # print("robot_paths: ", robot_paths)

        # print("\n matched list")
        # for robot_id, assignment in task_assignments.items():
        #     print(f"Robot {robot_id} taskID: {assignment['task_id']}, StartID: {assignment['start_id']},goalID: {assignment['goal_id']}")

        # SoC = self.calculate_total_path_length(robot_paths)
        total_SoC = 0
        for round_data in all_rounds_data:
            total_SoC += self.calculate_total_path_length(round_data['paths'])

        try:
            init()
            print(Fore.GREEN + "********************** The sum of cost are : {} ******************************".format(total_SoC) + Style.RESET_ALL, flush=True)
        except Exception:
            # fallback to ANSI if colorama not installed
            print("\033[92m********************** The sum of cost are : {} ******************************\033[0m".format(total_SoC), flush=True)
        time.sleep(5)
        # self.board.show_robots(robots, robot_paths, filename='{}/{}'.format(fig_save_dir, 0), plt_show=False)
        # print(len(robot_paths), robot_num)
        # if len(robot_paths) != robot_num:
        #     raise ValueError("Not all robots have a planned path")
        # for robot_id in robots:
        #     robots[robot_id].set_path(robot_paths[robot_id])


        # ===============================  任务执行 ===========================
        # moving_robots_num = 0
        # max_steps = 10000
        # for timestep in tqdm(range(max_steps)):
        #     moving_robots_num = len([robot for robot in robots.values() if len(robot.path()) > 0])
        #     if moving_robots_num == 0:
        #         print("All robots have reached their goals!")
        #         break
        #     logger.info('Timestep {}: '.format(timestep) + str(executor))
        #     executor.execute_actions()
        #
        #     # 注意：这里是可视化部分，可能运行较慢。可以直接注释掉/减少可视化元素以提高运行速度
        #     self.board.show_robots(robots, robot_paths,
        #                            filename='{}/{}'.format(fig_save_dir, timestep + 1), plt_show=False)



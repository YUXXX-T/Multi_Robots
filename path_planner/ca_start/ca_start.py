
import networkx as nx
from typing import Tuple, List, Dict, Set, Optional
from heapq import heappush, heappop
from map_module.map import Map
from path_planner.path_planner import PathPlanner
from stastar.planner import Planner


class Node:
    """搜索節點"""
    def __init__(self, position: int, time: int, g_cost: float, h_cost: float, parent: Optional['Node'] = None):
        self.position = position
        self.time = time
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.parent = parent

    @property
    def f_cost(self) -> float:
        return self.g_cost + self.h_cost

    def __lt__(self, other):
        return self.f_cost < other.f_cost


class CooperativeAStar(PathPlanner):
    def __init__(self, map_input: Map):
        super().__init__(map_input)
        self.__G = map_input.G

        # 核心數據結構-三層分離形式
        # 1. 上一輪機器人的終態信息
        self.__previous_round_info: Dict[int, Tuple[int, int]] = {}  # {robot_id: (end_position, end_time)}
        # 2. 當前論的路徑（規劃完成後清空）
        self.__current_round_paths: Dict[int, List[Tuple[int, int]]] = {}
        # 3. 時空占用表索引（用於快碰撞檢測）
        self.__spacetime_occupied: Dict[Tuple[int, int], int] = {}  # {(position, time): robot_id}

        # 全局時間偏移
        self.__global_time_offset = 0
        # 預計算最短距離（用於啓發式計算）
        self.__shortest_distances = self.__precompute_shortest_distances()

    # 狀態管理
    def set_state(self, reservation_table: Dict[Tuple[int, int], int], time_offset: int):
        """設置狀態（用於外部調用）"""
        self.__spacetime_occupied = reservation_table.copy()
        self.__global_time_offset = time_offset

        # 從reservation_table重建previous_round_info
        self.__previous_round_info.clear()
        robot_last_state = {}
        for (pos, t), rid in reservation_table.items():
            if rid not in robot_last_state or t > robot_last_state[rid][1]:
                robot_last_state[rid] = (pos, t)
        self.__previous_round_info = robot_last_state

    def get_reservation_table(self) -> Dict[Tuple[int, int], int]:
        """獲取預定表"""
        return self.__spacetime_occupied.copy()
    def get_time_offset(self) -> int:
        """獲取時間偏移"""
        return self.__global_time_offset
    def clear_reservation_table(self):
        """清空所有狀態"""
        self.__spacetime_occupied.clear()
        self.__previous_round_info.clear()
        self.__current_round_paths.clear()
        self.__global_time_offset = 0

    # 預計算和啓發式計算
    def __precompute_shortest_distances(self) -> Dict[Tuple[int, int], float]:
        """預計算所有節點對之間的最短距離"""
        try:
            all_pairs = dict(nx.all_pairs_dijkstra_path_length(self.__G, weight='weight'))
            distances = {}
            for source, targets in all_pairs.items():
                for target, length in targets.items():
                    distances[(source, target)] = length
            return distances
        except Exception as e:
            print(f"Waring！無法預計算最短距離：{e}")
            return {}

    def __heuristic(self, current: int, goal: int) -> float:
        """啓發式函數"""
        if (current, goal) in self.__shortest_distances:
            return self.__shortest_distances[(current, goal)]
        return 0

    # 碰撞檢測
    def _is_valid_move(self, from_pos: int, to_pos: int, time: int, robot_id: int) -> bool:
        """檢查移動是否有效"""
        # 檢查在地圖的合法性
        if from_pos != to_pos:
            if not self.__G.has_edge(from_pos, to_pos):
                return False
        # 檢查頂點衝突
        if (to_pos, time) in self.__spacetime_occupied:
            if self.__spacetime_occupied[(to_pos, time)] != robot_id:
                return False
        # 檢查邊衝突（交換位置）
        if (from_pos, time) in self.__spacetime_occupied and (to_pos, time - 1) in self.__spacetime_occupied:
            robot_at_from = self.__spacetime_occupied[(from_pos, time)]
            robot_at_to_prev = self.__spacetime_occupied[(to_pos, time - 1)]
            if robot_at_from == robot_at_to_prev and robot_at_from != robot_id:
                return False
        # 檢查跟隨衝突
        if (to_pos, time - 1) in self.__spacetime_occupied:
            prev_occupant = self.__spacetime_occupied[(to_pos, time - 1)]
            if prev_occupant != robot_id:
                if (to_pos, time) in self.__spacetime_occupied:
                    if self.__spacetime_occupied[(to_pos, time)] == prev_occupant:
                        return False
        return True

    # 路徑重建
    def _reconstruct_path(self, node: Node) -> List[Tuple[int, int]]:
        """從目標節點回溯構建路徑"""
        path = []
        current = node
        while current is not None:
            path.append((current.position, current.time))
            current = current.parent
        return path[::-1]

    # 單機器人規劃
    def _plan_single_robot_with_offset(self, start: int, goal: int, robot_id: int,
                                       max_time: int = 1000) -> Optional[List[Tuple[int, int]]]:
        """為單個機器人構造路徑（考慮全局時間偏移）"""
        if start not in self.__G.nodes or goal not in self.__G.nodes:
            return None
        start_node = Node(
            position=start,
            time=self.__global_time_offset,
            g_cost=0,
            h_cost=self.__heuristic(start, goal)
        )

        open_list = []
        heappush(open_list, start_node)
        closed_set: Set[Tuple[int, int]] = set()

        while open_list:
            current = heappop(open_list)

            if current.position == goal:
                return self._reconstruct_path(current)

            state = (current.position, current.time)
            if state in closed_set:
                continue
            closed_set.add(state)

            if current.time >= self.__global_time_offset + max_time:
                continue

            # 移動到四鄰域
            neighbors = list(self.__G.neighbors(current.position))
            for neighbor in neighbors:
                if self._is_valid_move(current.position, neighbor, current.time + 1, robot_id):
                    edge_data = self.__G.get_edge_data(current.position, neighbor)
                    edge_cost = edge_data.get('weight', 1) if edge_data else 1

                    new_node = Node(
                        position=neighbor,
                        time=current.time + 1,
                        g_cost=current.g_cost + edge_cost,
                        h_cost=self.__heuristic(neighbor, goal),
                        parent=current
                    )
                    heappush(open_list, new_node)

            # 等待動作
            if self._is_valid_move(current.position, current.position, current.time + 1, robot_id):
                wait_node = Node(
                    position=current.position,
                    time=current.time + 1,
                    g_cost=current.g_cost + 1.5,
                    h_cost=self.__heuristic(current.position, goal),
                    parent=current
                )
                heappush(open_list, wait_node)

        return None

    # 檢查衝突處理函數
    def _check_conflict(self, path: List[Tuple[int, int]], robot_id: int,
                        previous_round_robots: Dict[int, Tuple[int, int]],
                        current_round_paths: Dict[int, List[Tuple[int, int]]]) -> Optional[Dict]:
        """
        檢查路徑衝突
        返回: None(无冲突) 或 衝突信息
        """
        path_end_time = path[-1][1]

        # 收集可能衝突的位置
        conflict_positions = set()
        # 1. 上一輪機器人的終點（如果時間更短）
        for prev_rid, (prev_pos, prev_time) in previous_round_robots.items():
            if prev_time < path_end_time:
                conflict_positions.add(prev_pos)

        # 2. 當前輪中已經規劃機器人的終點（如果時間更短）
        for other_rid, other_path in current_round_paths.items():
            other_end_time = other_path[-1][1]
            other_end_pos = other_path[-1][0]
            if other_end_time < path_end_time:
                conflict_positions.add(other_end_pos)

        # 檢查路徑是否經過這些衝突位置
        for pos, t in path:
            if pos in conflict_positions:
                conflicting_robots = []

                # 找到衝突終點屬於的機器人id
                for prev_rid, (prev_pos, prev_time) in previous_round_robots.items():
                    if prev_pos == pos and prev_time < t:
                        conflicting_robots.append(('previous', prev_rid, prev_pos, prev_time))

                for other_rid, other_path in current_round_paths.items():
                    other_end_time = other_path[-1][1]
                    other_end_pos = other_path[-1][0]
                    if other_end_pos == pos and other_end_time < t:
                        conflicting_robots.append(('current', other_rid, other_end_pos, other_end_time))

                if conflicting_robots:
                    return {
                        'position': pos,
                        'time': t,
                        'conflicting_robots': conflicting_robots
                    }

        return None

    def _extend_conflicting_robots(self, conflict: Dict, target_time: int,
                                   previous_round_robots: Dict[int, Tuple[int, int]],
                                   current_round_paths: Dict[int, List[Tuple[int, int]]]):
        """延長發生衝突的機器人"""
        extended_count = 0

        for robot_type, rid, pos, current_end_time in conflict['conflicting_robots']:
            print(f"      延長 {robot_type} 機器人 {rid}: {current_end_time} -> {target_time}")

            for t in range(current_end_time + 1, target_time + 1):
                if (pos, t) not in self.__spacetime_occupied:
                    self.__spacetime_occupied[(pos, t)] = rid
                    extended_count += 1

            # 更新記錄
            if robot_type == 'previous':
                previous_round_robots[rid] = (pos, target_time)
            else:
                # 延長當前輪路徑
                for t in range(current_end_time + 1, target_time + 1):
                    current_round_paths[rid].append((pos, t))
        print(f"      共延長 {extended_count} 個時空點")

    def _align_all_robots(self, previous_round_robots: Dict[int, Tuple[int, int]],
                          current_round_paths: Dict[int, List[Tuple[int, int]]]):
        """對齊所有機器人到最大時間"""
        if not current_round_paths:
            return

        max_time = max(p[-1][1] for p in current_round_paths.values())

        # 對齊上一輪機器人
        for rid, (pos, end_time) in previous_round_robots.items():
            for t in range(end_time + 1, max_time + 1):
                if (pos, t) not in self.__spacetime_occupied:
                    self.__spacetime_occupied[(pos, t)] = rid

        # 對齊當前輪機器人
        for rid, path in current_round_paths.items():
            end_pos = path[-1][0]
            end_time = path[-1][1]
            for t in range(end_time + 1, max_time + 1):
                if (end_pos, t) not in self.__spacetime_occupied:
                    self.__spacetime_occupied[(end_pos, t)] = rid
                    path.append((end_pos, t))

    def _occupy_spacetime(self, path: List[Tuple[int, int]], robot_id: int):
        """占用時空表"""
        for pos, t in path:
            self.__spacetime_occupied[(pos, t)] = robot_id

    def _validate_all_paths(self, all_paths: Dict[int, List[Tuple[int, int]]]) -> Optional[Dict]:
        """
        驗證所有路徑是否有衝突
        返回: None(無衝突) 或者 衝突描述字符串
        """
        # 1. 檢查頂點衝突
        spacetime_occupation: Dict[Tuple[int, int], int] = {}

        for robot_id, path in all_paths.items():
            for pos, t in path:
                if (pos, t) in spacetime_occupation:
                    other_robot = spacetime_occupation[(pos, t)]
                    return {
                        'type': 'vertex',
                        'robot1': robot_id,
                        'robot2': other_robot,
                        'position': pos,
                        'time': t,
                        'description': f"頂點衝突: 機器人 {robot_id} 和 {other_robot} 在時間 {t} 都占用位置 {pos}"
                        }
                spacetime_occupation[(pos, t)] = robot_id

        # 2. 檢查邊衝突（交換位置）
        for robot_id, path in all_paths.items():
            for i in range(len(path) - 1):
                pos1, t1 = path[i]
                pos2, t2 = path[i + 1]

                # 檢查是否有其他機器人在同一時間从pos2移動到pos1
                for other_id, other_path in all_paths.items():
                    if other_id == robot_id:
                        continue
                    for j in range(len(other_path) - 1):
                        other_pos1, other_t1 = other_path[j]
                        other_pos2, other_t2 = other_path[j + 1]
                        # 邊衝突：两個機器人交換位置
                        if (pos1 == other_pos2 and pos2 == other_pos1 and
                                t1 == other_t1 and t2 == other_t2):
                                return {
                                    'type': 'edge',
                                    'robot1': robot_id,
                                    'robot2': other_id,
                                    'edge': (pos1, pos2),
                                    'time': t1,
                                    'description': f"邊衝突: 機器人 {robot_id} 和 {other_id} 在時間 {t1}->{t2} 交換位置 {pos1}<->{pos2}"
                                }
        return None

    def _replan_robot_round(self,
                          robot_id: int,
                          start: int,
                          goal: int,
                          previous_round_robots: Dict[int, Tuple[int, int]],
                          *,
                          is_first_robot: bool = False,
                          max_replan: int = 10) -> Optional[List[Tuple[int, int]]]:
        """
        把plan()中“單個機器人一整輪”的重規劃流程獨立封裝：
            -反復A*規劃
            -首機器人首輪強制延長上一輪
            -衝突檢測與對方延長
            -接受路徑->佔用時空錶->立刻對齊到當前最大時間
        接受成功返回路徑，否則返回None。
        會直接修改：
            - self.__current_round_paths
            - self.__spacetime_occupied
            - previous_round_robots（若發生延長）
        """
        for replan_count in range(max_replan):
            # 1. 規劃
            path = self._plan_single_robot_with_offset(start, goal, robot_id)
            if path is None:
                print("   無法找到路徑！")
                return None

            path_end_time = path[-1][1]
            if replan_count == 0:
                print(f"   初始路徑: 長度 {len(path)}, 結束時間 {path_end_time}")
            else:
                print(f"   第 {replan_count} 次重規劃: 長度 {len(path)}, 結束時間 {path_end_time}")

            # 2) 首個機器人首輪：强制把上一輪全部延長到該路徑結束時間，然後重新規劃
            if is_first_robot and replan_count == 0 and previous_round_robots:
                print("\n   第一個機器人强制延長上一輪")
                extended_count = 0
                for prev_rid, (prev_pos, prev_time) in previous_round_robots.items():
                    for t in range(prev_time + 1, path_end_time + 1):
                        if (prev_pos, t) not in self.__spacetime_occupied:
                            self.__spacetime_occupied[(prev_pos, t)] = prev_rid
                            extended_count += 1
                    previous_round_robots[prev_rid] = (prev_pos, path_end_time)
                print(f"   延長了 {extended_count} 個時空點")
                print("   强制重新規劃...")
                continue

            # 3. 衝突檢查
            conflict = self._check_conflict(
                path,
                robot_id,
                previous_round_robots,
                self.__current_round_paths
            )
            if conflict:
                print(f"   檢測到衝突: 位置 {conflict['position']}, 時間 {conflict['time']}")
                # 延長衝突機器人到當前路徑終點，再重規劃
                self._extend_conflicting_robots(
                    conflict,
                    path_end_time,
                    previous_round_robots,
                    self.__current_round_paths
                )
                continue

            # 4) 無衝突則 接受路徑、占用、立刻對齊
            print("   路徑有效")
            self.__current_round_paths[robot_id] = path
            self._occupy_spacetime(path, robot_id)

            # 對齊必須在接受後立刻運行，否则下一台機器人可能基於未對齊的狀態規劃
            self._align_all_robots(previous_round_robots, self.__current_round_paths)
            return path

        print("   重規劃次數超過限額！")
        return None



    # 規劃函數
    def plan(self, robot_start_id_dict: Dict[int, int],
             robot_goal_id_dict: Dict[int, int],
             priority_order: Optional[List[int]] = None) -> Dict[int, List[Tuple[int, int]]]:
        """CA*函數"""

        # 1. 準備階段
        if set(robot_start_id_dict.keys()) != set(robot_goal_id_dict.keys()):
            print("Error! 起點與終點數目不匹配！")
            return {}

        if priority_order is None:
            robot_ids = sorted(robot_start_id_dict.keys())
        else:
            robot_ids = priority_order
            if set(robot_ids) != set(robot_start_id_dict.keys()):
                print("Error! 優先級列表不匹配！")
                return {}

        # 清空當前輪規劃結構
        self.__current_round_paths.clear()

        # 識別上一輪機器人
        current_robot_set = set(robot_ids)
        previous_round_robots = {
            rid: info for rid, info in self.__previous_round_info.items()
            if rid not in current_robot_set
        }

        print(f"\n{'=' * 60}")
        print(f"開始新一輪規劃")
        print(f"當前機器人: {robot_ids}")
        if previous_round_robots:
            print(f"上一輪機器人: {list(previous_round_robots.keys())}")
            for rid, (pos, t) in previous_round_robots.items():
                print(f"  機器人 {rid}: 位置 {pos}, 時間 {t}")
        print(f"全局時間偏移: {self.__global_time_offset}")
        print(f"{'=' * 60}\n")

        # 2. 單機器人逐步規劃
        for idx, robot_id in enumerate(robot_ids):
            start = robot_start_id_dict[robot_id]
            goal = robot_goal_id_dict[robot_id]
            print(f"\n[{idx + 1}/{len(robot_ids)}] 規劃機器人 {robot_id}: {start} -> {goal}")

            is_first_robot = (idx == 0)

            # # 重規劃循環
            # for replan_count in range(10):
            #     # 規劃路徑
            #     path = self._plan_single_robot_with_offset(start, goal, robot_id)
            #     if path is None:
            #         print(f"   無法找到路徑！")
            #         return {}
            #
            #     path_end_time = path[-1][1]
            #     if replan_count == 0:
            #         print(f"   初始路徑: 長度 {len(path)}, 結束時間 {path_end_time}")
            #     else:
            #         print(f"   第 {replan_count} 次重規劃: 長度 {len(path)}, 結束時間 {path_end_time}")
            #     # 第一個機器人强制延長上一輪
            #     if is_first_robot and replan_count == 0 and previous_round_robots:
            #         print(f"\n   第一個機器人强制延長上一輪")
            #         extended_count = 0
            #         for prev_rid, (prev_pos, prev_time) in previous_round_robots.items():
            #             for t in range(prev_time + 1, path_end_time + 1):
            #                 if (prev_pos, t) not in self.__spacetime_occupied:
            #                     self.__spacetime_occupied[(prev_pos, t)] = prev_rid
            #                     extended_count += 1
            #             # 更新previous_round_robots
            #             previous_round_robots[prev_rid] = (prev_pos, path_end_time)
            #
            #         print(f"   延長了 {extended_count} 個時空點")
            #         print(f"   强制重新規劃...")
            #         continue  #  跳過衝突檢查 直接重規劃
            #
            #     # 衝突檢查
            #     conflict = self._check_conflict(
            #         path,
            #         robot_id,
            #         previous_round_robots,
            #         self.__current_round_paths
            #     )
            #     if conflict:
            #         print(f"   檢測到衝突: 位置 {conflict['position']}, 時間 {conflict['time']}")
            #         # 延長衝突的機器人
            #         self._extend_conflicting_robots(
            #             conflict,
            #             path_end_time,
            #             previous_round_robots,
            #             self.__current_round_paths
            #         )
            #         continue  # 重新規劃
            #
            #     # 無衝突 接受路徑
            #     print(f"   路徑有效")
            #     self.__current_round_paths[robot_id] = path
            #     self._occupy_spacetime(path, robot_id)
            #     break
            #
            # else:  # for-else
            #     print(f"   重規劃次數超過限額！")
            #     return {}
            #
            # # 對齊所有機器人
            # self._align_all_robots(previous_round_robots, self.__current_round_paths)
            path = self._replan_robot_round(
                robot_id,
                start,
                goal,
                previous_round_robots,
                is_first_robot=is_first_robot,
                max_replan=10
            )
            if path is None:
                return {}

        # 3. 最終驗證階段
        print(f"\n{'=' * 60}")
        print(f"最終路徑驗證...")
        conflict_msg = self._validate_all_paths(self.__current_round_paths)
        if conflict_msg:
        #     # TODO等待策略
            conflict_robot1 = conflict_msg['robot1']
            conflict_robot2 = conflict_msg['robot2']
            replan_robotid = min(conflict_robot1, conflict_robot2)
            old_path = self.__current_round_paths.pop(replan_robotid, None)
            if old_path:
                removed_count = 0
                for pos, t in old_path:
                    # 只刪除屬於該機器人的佔用
                    if self.__spacetime_occupied.get((pos, t)) == replan_robotid:
                        del self.__spacetime_occupied[(pos, t)]
                        removed_count += 1
                print(f"   已移除機器人 {replan_robotid} 的 {removed_count} 個時空佔用點")

            new_plan = self._replan_robot_round(
                replan_robotid,
                robot_start_id_dict[replan_robotid],
                robot_goal_id_dict[replan_robotid],
                previous_round_robots,
                is_first_robot = False,
                max_replan = 10
            )
            if new_plan is None:
                print("   最終驗證階段重規劃失敗，終止。")
                return {}


        # 4. 收尾階段
        # 更新 previous_round_info（为下一輪做準備）
        for rid, path in self.__current_round_paths.items():
            self.__previous_round_info[rid] = (path[-1][0], path[-1][1])

        # 更新全局時間偏移
        if self.__current_round_paths:
            max_time = max(p[-1][1] for p in self.__current_round_paths.values())
            self.__global_time_offset = max_time

        print(f"\n{'=' * 60}")
        print(f"  本輪規劃完成")
        print(f"  時空占用點數: {len(self.__spacetime_occupied)}")
        print(f"  全局時間偏移: {self.__global_time_offset}")
        print(f"{'=' * 60}\n")

        return self.__current_round_paths

class CAstarPlanner(PathPlanner):
    def __init__(self, map_input: Map):
        super().__init__(map_input)
        self.__G = map_input.G

    def plan_path(self, start: int, goal: int) -> List[int]:
        """單機器人路徑規劃"""
        path = []
        try:
            path = nx.shortest_path(self.__G, start, goal)
        except nx.NetworkXNoPath:
            print('Warning: No Path Found!')
        return path

    def plan_paths_from_CAstar(self, starts: Dict[int, int], goals: Dict[int, int],
                               priority_order: Optional[List[int]] = None) -> Dict[int, List[int]]:
        """使用CA*規劃多機器人路徑"""
        if not hasattr(self, '_ca_instance'):
            self._ca_instance = CooperativeAStar(self._map)

        ca = self._ca_instance
        time_paths = ca.plan(starts, goals, priority_order)

        if not time_paths:
            return {}

        # 轉換爲節點ID序列（去重）
        out: Dict[int, List[int]] = {}
        for rid, pt in time_paths.items():
            seq = [p for (p, _) in pt]
            if seq:
                comp = [seq[0]]
                for x in seq[1:]:
                    if x != comp[-1]:
                        comp.append(x)
                seq = comp
            out[rid] = seq
        return out

    def plan_paths_with_state(self, robot_start_id_dict: Dict[int, int],
                              robot_goal_id_dict: Dict[int, int],
                              reservation_table: Dict[Tuple[int, int], int],
                              time_offset: int,
                              priority_order: Optional[List[int]] = None,
                              delivery: bool = False) \
            -> Tuple[Dict[int, List[int]], Dict[Tuple[int, int], int], int]:
        """帶狀態的路徑規劃"""
        ca = CooperativeAStar(self._map)
        ca.set_state(reservation_table.copy(), time_offset)

        time_paths = ca.plan(robot_start_id_dict, robot_goal_id_dict, priority_order)

        if not time_paths:
            return {}, reservation_table, time_offset

        # 獲取更新後的狀態
        updated_table = ca.get_reservation_table()
        new_offset = ca.get_time_offset()

        # 轉換為節點ID序列
        out: Dict[int, List[int]] = {}
        for rid, pt in time_paths.items():
            seq = [p for (p, _) in pt]
            if seq:
                comp = [seq[0]]
                for x in seq[1:]:
                    if x != comp[-1]:
                        comp.append(x)
                seq = comp
            out[rid] = seq

        return out, updated_table, new_offset



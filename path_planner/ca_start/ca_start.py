import networkx as nx
from typing import Tuple, List, Dict
from map_module.map import Map
from path_planner.path_planner import PathPlanner
from stastar.planner import Planner

from typing import Dict, List, Tuple, Set, Optional
from heapq import heappush, heappop


class Node:
    """
    搜索節點

    position: int   # 當前位置ID
    time: int       # 當前時間步
    g_cost: float   # 從起點到當前節點的實際代價
    h_cost: float   # 從當前節點到終點的啟發式估計代價
    parent: Optional['Node'] = None   # 父節點，用於回溯路徑
    """
    def __init__(self, position: int, time: int, g_cost: float, h_cost: float, parent: Optional['Node'] = None):
        self.position = position
        self.time = time
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.parent = parent

    @property
    def f_cost(self) -> float:
        """ 縂代價為： f = g + h """
        return self.g_cost + self.h_cost

    def __lt__(self, other):
        """ 用於優先隊列的比較 """
        return self.f_cost < other.f_cost

class CooperativeAStar(PathPlanner):
    def __init__(self, map_input: Map):
        super().__init__(map_input)
        self.__G = map_input.G

        # 用時空表記錄每個(位置, 時間)被哪個機器人占用
        # 格式: {(position, time): robot_id}
        self.__reservation_table: Dict[Tuple[int, int], int] = {}

        # 預計算所有節點對之間的最短距離（用於啟發式函數）
        # 這會在初始化時計算一次，提高後續規劃效率
        self.__shortest_distances = self.__precompute_shortest_distances()

        # 全局時間位移
        self.__global_time_offset = 0  # 記錄當前是第幾輪規劃

    def set_state(self, reservation_table: Dict[Tuple[int, int], int], time_offset: int):
        self.__reservation_table = reservation_table.copy()
        self.__global_time_offset = time_offset

    def get_reservation_table(self) -> Dict[Tuple[int, int], int]:
        return self.__reservation_table

    def get_time_offset(self) -> int:
        return self.__global_time_offset

    def __precompute_shortest_distances(self) -> Dict[Tuple[int, int], float]:
        """
        預計算所有節點對之間的最短距離
        返回:
             最短距離字典 {(node1, node2): distance}
        說明:
            使用Floyd-Warshall或多次Dijkstra算法計算
            這個預計算能顯著提高啟發式函數的準確性和速度
        """
        try:
            # 這個用的是什麽算？
            # all_pairs = dict(nx.all_pairs_shortest_path_length(self.__G))
            all_pairs = dict(nx.all_pairs_dijkstra_path_length(self.__G, weight='weight'))

            distances = {}
            for source, targets in all_pairs.items():
                for target, length in targets.items():
                    distances[(source, target)] = length
            return distances

        except Exception as e:
            print(f"警告：無法預計算最短距離：{e}")
            return {}

    def __heuristic(self, current: int, goal: int) -> float:
        """
        啟發式函數：估計從當前位置到目標位置的距離
        參數:
            current: 當前位置ID
            goal: 目標位置ID
        返回:
            估計距離
        說明:
            使用預計算的最短距離作為啟發式值
            這是一個可接受的（admissible）啟發式，保證找到最優解
        """
        # 如果預計算成功，使用精確的最短距離   爲什麽要預計？
        if (current, goal) in self.__shortest_distances:
            return self.__shortest_distances[(current, goal)]
        # 否則返回0（退化為Dijkstra算法）
        return 0

    def _is_valid_move(self, from_pos: int, to_pos: int, time: int, robot_id: int) -> bool:
        """
        檢查移動是否有效（避免衝突）
        參數:
            from_pos: 起始位置      to_pos: 目標位置    time: 到達目標位置的時間     robot_id: 當前機器人ID
        返回:
            True如果移動有效，False如果有衝突

        需要檢查的衝突類型:
        1. 頂點衝突 (Vertex Conflict): 兩個機器人在同一時間占用同一位置
        2. 邊衝突 (Edge Conflict): 兩個機器人在同一時間交換位置
        3. 圖結構約束: 確保移動在圖中是合法的
        """
        # 檢查移動是否在圖結構中合法    圖結構合法是什麽？
        if from_pos != to_pos:
            if not self.__G.has_edge(from_pos, to_pos):
                return False  # 圖中不存在這條邊
        # 檢查目標位置在目標時間是否被占用（頂點衝突）
        if (to_pos, time) in self.__reservation_table:
            occupying_robot = self.__reservation_table[(to_pos, time)]
            if occupying_robot != robot_id:
                return False

        # 檢查邊衝突：其他機器人是否在同一時間從to_pos移動到from_pos
        if (from_pos, time) in self.__reservation_table and (to_pos, time-1) in self.__reservation_table:
            robot_at_from = self.__reservation_table[(from_pos, time)]
            robot_at_to_prev = self.__reservation_table[(to_pos, time-1)]
            # 如果有機器人在time時刻占用from_pos，
            # 且在time-1時刻占用to_pos，則發生交叉衝突
            if robot_at_from == robot_at_to_prev and robot_at_from != robot_id:
                return False
        # 检查同向跟随冲突
        # 如果前面的机器人在time-1时刻在to_pos，在time时刻也在to_pos（等待）
        # 不能在time时刻进入to_pos
        if (to_pos, time-1) in self.__reservation_table:
            prev_occupant = self.__reservation_table[(to_pos, time-1)]
            if prev_occupant != robot_id:
                if (to_pos, time) in self.__reservation_table:
                    if self.__reservation_table[(to_pos, time)] == prev_occupant:
                        return False
            # 如果 to_pos 在 time-1 被占用，且在 time 仍被同一机器人占用，说明那个机器人在等待
            if (to_pos, time - 1) in self.__reservation_table and (to_pos, time) in self.__reservation_table:
                prev_robot = self.__reservation_table[(to_pos, time - 1)]
                curr_robot = self.__reservation_table[(to_pos, time)]

                if prev_robot == curr_robot and prev_robot != robot_id:
                    # 有其他机器人在 to_pos 等待，不能进入
                    return False

            # 检查是否有机器人从 to_pos 等待到未来
            # 扫描 to_pos 的未来时间片段
            if (to_pos, time) in self.__reservation_table:
                future_occupant = self.__reservation_table[(to_pos, time)]
                if future_occupant != robot_id:
                    # 检查这个机器人是否从过去就在等待
                    check_time = time - 1
                    while check_time >= 0:
                        if (to_pos, check_time) not in self.__reservation_table:
                            break
                        if self.__reservation_table[(to_pos, check_time)] != future_occupant:
                            break
                        check_time -= 1
                    else:
                        # 说明future_occupant一直在to_pos等待
                        return False
        return True

    def _reconstruct_path(self, node: Node) -> List[Tuple[int, int]]:
        """
        從目標節點回溯構建完整路徑
        參數:
            node: 目標節點
        返回:
            路徑列表，格式 [(position, time), ...]
        """
        path = []
        current = node

        while current is not None:
            path.append((current.position, current.time))
            current = current.parent
        return path[::-1]

    def _reserve_path(self, path: List[Tuple[int, int]], robot_id: int):
        """
        在預定表中預定路徑
        參數:
            path: 路徑列表 [(position, time), ...]
            robot_id: 機器人ID
        說明:
            將機器人的整條路徑加入預定表
            後續機器人規劃時會查詢這個表來避免衝突
        """

        for position, time in path:
            self.__reservation_table[(position, time)] = robot_id

    def _reserve_goal_indefinitely(self, all_paths: Dict[int, List[Tuple[int, int]]]):
        if not all_paths:
            return

        max_time = max(path[-1][1] for path in all_paths.values())
        # 为每个机器人预定目标位置到最大时间+安全边界
        safety_margin = 10  # 安全边界，可调整
        for robot_id, path in all_paths.items():
            final_pos = path[-1][0]
            final_time = path[-1][1]

            # 预定从完成时间到最大时间+安全边界
            for t in range(final_time + 1, max_time + safety_margin):
                self.__reservation_table[(final_pos, t)] = robot_id



    def _plan_single_robot(self, start: int, goal: int, robot_id: int, max_time: int = 1000) -> Optional[List[Tuple[int, int]]]:
        """
        為單個機器人規劃路徑（基於時空A*算法）
        參數:
            start: 起始位置ID
            goal: 目標位置ID
            robot_id: 機器人ID
            max_time: 最大搜索時間步（防止無限循環）
        返回:
            路徑列表或None（如果找不到路徑）
        算法流程:
            1. 初始化起始節點，加入open list
            2. 從open list取出f_cost最小的節點
            3. 如果到達目標，回溯路徑
            4. 否則擴展鄰居節點（包括等待動作）
            5. 重複2-4直到找到路徑或open list為空
        """
        # 檢查起點和終點是否在圖中
        if start not in self.__G.nodes or goal not in self.__G.nodes:
            print(f"Wrong with start: {start}, goal: {goal}")
            return None

        # 初始化起始節點
        start_node = Node(
            position=start,
            time=0,
            g_cost=0,
            h_cost=self.__heuristic(start, goal)
        )

        # openlist: 待探索的節點
        open_list = []
        heappush(open_list, start_node)

        # close set: 已探索的(位置, 時間)對
        closed_set: Set[Tuple[int, int]] = set()

        while open_list:
            # 取出f_cost最小的節點 自動地
            current = heappop(open_list)

            if current.position == goal:
                return self._reconstruct_path(current)

            state = (current.position, current.time)
            # 如果已探索過，跳過
            if state in closed_set:
                continue
            # 標記為已探索
            closed_set.add(state)

            if current.time >= max_time:
                continue

            # 探索所有可能的動作
            # 動作1: 移動到相鄰節點
            # 使用NetworkX獲取鄰居節點, 不出意外就是四領域
            neighbors = list(self.__G.neighbors(current.position))
            for neighbor in neighbors:
                if self._is_valid_move(current.position, neighbor, current.time+1, robot_id):
                    # 獲取邊的權重（如果有的話）  這是怎麽計算的？
                    edge_data = self.__G.get_edge_data(current.position, neighbor)
                    edge_cost = edge_data.get('weight', 1) if edge_data else 1

                    new_node = Node(
                        position = neighbor,
                        time = current.time+1,
                        g_cost = current.g_cost + edge_cost,
                        h_cost = self.__heuristic(neighbor, goal),
                        parent = current

                    )
                    heappush(open_list, new_node)

            # 動作2: 等待動作（停留在原地）
            # 這對於避讓其他機器人很重要
            if self._is_valid_move(current.position, current.position, current.time+1, robot_id):
                wait_node = Node(
                    position = current.position,
                    time = current.time + 1,
                    g_cost = current.g_cost +1, # 等待代價
                    h_cost = self.__heuristic(current.position, goal),
                    parent = current
                )
                heappush(open_list, wait_node)

        return None

    def _plan_single_robot_with_offset(self, start: int, goal: int, robot_id: int,
                                       max_time: int = 1000, period_offset: int = 0) -> Optional[List[Tuple[int, int]]]:
        """
        为单个机器人规划路径（考虑全局时间偏移）
        """
        if start not in self.__G.nodes or goal not in self.__G.nodes:
            print(f"Wrong with start: {start}, goal: {goal}")
            return None

        # 起始時間使用全局偏移
        start_node = Node(
            position=start,
            time=self.__global_time_offset+period_offset,
            g_cost=0,
            h_cost=self.__heuristic(start, goal)
        )

        # openlist: 待探索的節點
        open_list = []
        heappush(open_list, start_node)

        # close set: 已探索的(位置, 時間)對
        closed_set: Set[Tuple[int, int]] = set()

        while open_list:
            # 取出f_cost最小的節點 自動地
            current = heappop(open_list)

            if current.position == goal:
                return self._reconstruct_path(current)

            state = (current.position, current.time)
            # 如果已探索過，跳過
            if state in closed_set:
                continue
            # 標記為已探索
            closed_set.add(state)

            if current.time >= self.__global_time_offset + max_time:
                continue

            # 探索所有可能的動作
            # 動作1: 移動到相鄰節點
            # 使用NetworkX獲取鄰居節點, 不出意外就是四領域
            neighbors = list(self.__G.neighbors(current.position))
            for neighbor in neighbors:
                if self._is_valid_move(current.position, neighbor, current.time + 1, robot_id):
                    # 獲取邊的權重（如果有的話）  這是怎麽計算的？
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

            # 動作2: 等待動作（停留在原地）
            # 這對於避讓其他機器人很重要
            if self._is_valid_move(current.position, current.position, current.time + 1, robot_id):
                wait_node = Node(
                    position=current.position,
                    time=current.time + 1,
                    g_cost=current.g_cost + 1.5,  # 等待代價
                    h_cost=self.__heuristic(current.position, goal),
                    parent=current
                )
                heappush(open_list, wait_node)

        return None

    def plan(self,
             robot_start_id_dict: Dict[int, int],
             robot_goal_id_dict: Dict[int, int],
             priority_order: Optional[List[int]] = None) -> Dict[int, List[Tuple[int, int]]]:
        """
        CA*主函數：為所有機器人規劃協作路徑
        參數:
            robot_start_id_dict: 機器人起始位置字典 {robot_id: start_position}
            robot_goal_id_dict: 機器人目標位置字典 {robot_id: goal_position}
            priority_order: 可選的優先級順序列表，如果不提供則按robot_id排序
        返回:
            所有機器人的路徑字典 {robot_id: [(position, time), ...]}  如果規劃失敗返回空字典
        算法流程:
            1. 清空預定表
            2. 確定機器人優先級順序
            3. 按優先級順序為每個機器人規劃路徑
            4. 每規劃完一個機器人，將其路徑加入預定表
            5. 後續機器人規劃時會自動避開已預定的時空位置
        """
        # 保留 >= global_time_offset的預定表  __reservation_table : Dict[Tuple[pos, t], robot_id]
        # keys_to_remove = [k for k in self.__reservation_table.keys()
        #                   if k[1] < self.__global_time_offset]
        # for k in keys_to_remove:
        #     del self.__reservation_table[k]
        self.__reservation_table = {
            k: v for k, v in self.__reservation_table.items()
            if k[1] >= self.__global_time_offset
        }

        if set(robot_start_id_dict.keys()) != set(robot_goal_id_dict.keys()):
            print(f"Error with {len(robot_start_id_dict.keys())}")
            return {}

        all_paths: Dict[int, List[Tuple[int, int]]] = {}
        robot_real_end_info: Dict[int, Tuple[int, int]] = {}  # {robot_id: (end_pos, end_time)}
        if priority_order is None:
            robot_ids = sorted(robot_start_id_dict.keys())
        else:
            robot_ids = priority_order
            if set(robot_ids) != set(robot_start_id_dict.keys()):
                print(f"Error with priority order")
                return {}

        def get_max_reservation_time():
            return max((t for (pos, t) in self.__reservation_table.keys()), default=0)

        def get_robot_end_positions():
            max_time = get_max_reservation_time()
            end_positions = {}
            if max_time == 0:
                # 第一輪：使用起點
                for rid in robot_ids:
                    end_positions[rid] = robot_start_id_dict[rid]
            else:
                # 後續輪次：從預定表提取
                for (pos, t), rid in self.__reservation_table.items():
                    if t == max_time:
                        end_positions[rid] = pos
                # 補充當前輪次的機器人起點
                for rid in robot_ids:
                    if rid not in end_positions:
                        end_positions[rid] = robot_start_id_dict[rid]
            return end_positions
        # reservation_table 最大時間
        # max_reservation_table_time = max(
        #     t for (pos, t) in self.__reservation_table.keys()) \
        #     if self.__reservation_table else 0
        #
        # # 記錄每輪的機器人初始化位置
        # robot_real_end_pos: Dict[int, int] = {}  # {robot_id: pos}
        # if max_reservation_table_time == 0:
        #     for idx, robot_id in enumerate(robot_ids):
        #         robot_real_end_pos[robot_id] = robot_start_id_dict[robot_id]
        # else:
        #     robots_at_max_time = {}
        #     for (pos, t), robot_id in self.__reservation_table.items():
        #         if t == max_reservation_table_time:
        #             # robot_real_end_pos[robot_id] = pos
        #             robots_at_max_time[robot_id] = pos
        #     all_robot_ids = set(robot_start_id_dict.keys()) | set(robots_at_max_time.keys())
        #     for rid in all_robot_ids:
        #         if rid in robots_at_max_time:
        #             robot_real_end_pos[rid] = robots_at_max_time[rid]
        #         elif rid in robot_start_id_dict:
        #             robot_real_end_pos[rid] = robot_start_id_dict[rid]

        print(f"開始為 {len(robot_ids)} 個機器人規劃路徑...")
        print(f"優先級順序: {robot_ids}")
        print(f"全局時間偏移: {self.__global_time_offset}")
        print(f"預定表大小: {len(self.__reservation_table)}")

        for idx, robot_id in enumerate(robot_ids):
            # 規劃初始化
            start = robot_start_id_dict[robot_id]
            goal = robot_goal_id_dict[robot_id]

            print(f"\n[{idx + 1}/{len(robot_ids)}] 規劃機器人 {robot_id}: {start} -> {goal}")

            # 重規劃循環
            max_replans = 10
            found_valid_path = False
            final_path = None
            for replan_count in range(max_replans):
                # 每次规划前都获取最新状态
                current_max_res_time = get_max_reservation_time()
                robot_end_positions = get_robot_end_positions()
                # 規劃路徑
                path = self._plan_single_robot_with_offset(start, goal, robot_id)

                if path is None:
                    print(f"   錯誤：無法為機器人 {robot_id} 找到路徑！")
                    print(f"   可能原因：")
                    print(f"   1. 起點或終點不可達")
                    print(f"   2. 與其他機器人衝突無法解決")
                    print(f"   3. 搜索超時")
                    return {}

                current_path_end_time = path[-1][1]
                current_path_end_pos = path[-1][0]

                if replan_count == 0:
                    print(f"   路徑長度：{len(path)} 完成時間: {current_path_end_time}")
                else:
                    print(f"   第 {replan_count} 次重規劃：路徑長度 {len(path)} 完成時間: {current_path_end_time}")


                # 檢查之前機器人的終點是否在當前路徑上
                has_conflict = False

                # 同一輪的路徑規劃中，只需要考慮時間更長情況，時間更長才會產生衝突
                if current_max_res_time < current_path_end_time and current_max_res_time>0:
                    # if max_time < current_path_end_time or count_times == 0:
                    # count_times += 1 # 第一次的話，all_paths是空的
                    # 找出所有在 reservation_table 中的机器人终点位置
                    # 终点 = 在最大时间的位置

                    # ouccupied position是不是可以被robot_real_end_pos 替代？
                    occupied_positions = set(robot_end_positions.values())
                    # for (pos, t) in self.__reservation_table.keys():
                    #     if t == max_reservation_table_time:
                    #         occupied_positions.add(pos)

                    # 检查当前路径在 max_reservation_table_time 之后是否经过这些位置
                    for pos, t in path:
                        if t > current_max_res_time  and pos in occupied_positions:
                            print(f"   機器人 {robot_id}  路徑在時間 {t} 經過位置 {pos}（其他機器人的終點）")
                            has_conflict = True
                            break  # 衝突則停止此for循環

                if not has_conflict:
                    found_valid_path = True
                    final_path = path
                    break  # 跳出重規劃for循環 下方延長不運行

                # 如果有衝突，需要重新規劃
                print(f"\n   機器人 {robot_id} 需要重新規劃! ")

                extension_count = 0
                # 將所有機器人在reservation_table裏面的時間都延長到這條衝突路徑的終點時間
                for robid, robpo in robot_end_positions.items():
                    for nt in range(current_max_res_time + 1, current_path_end_time+1):
                        if (pos, nt) not in self.__reservation_table:
                            self.__reservation_table[(robpo, nt)] = robid
                            extension_count += 1

                print(f"   延長了 {extension_count} 個時空點")
                # for (pos, t) in list(self.__reservation_table.keys()):
                #     if t == max_reservation_table_time:
                #         # 找出占用這個位置的機器人
                #         occupying_robot = self.__reservation_table[(pos, t)]
                #         # 延長到當前路徑結束時間
                #         for new_t in range(max_reservation_table_time+1, current_path_end_time+1):
                #             if (pos, new_t) not in self.__reservation_table:
                #                 self.__reservation_table[(pos, new_t)] = occupying_robot
            # 規劃10個機器人路徑的for循環結束
            if not found_valid_path:
                print(f"   重新規劃次數超過限制！")
                return {}

            # 確認無衝突後，預定路徑
            self._reserve_path(final_path, robot_id)
            all_paths[robot_id] = final_path

            final_pos = final_path[-1][0]
            final_time = final_path[-1][1]
            # robot_real_end_info[robot_id] = (final_pos, final_time)
            # 更新robot_real_end_pos情況
            # robot_real_end_pos[robot_id] = final_pos
            print(f"   機器人 {robot_id} 路徑確認，真實結束: 位置{final_pos}, 時間{final_time}")

            # 延長所有已規劃機器人的終點到當前最大時間
            if all_paths:
                current_max_time = max(p[-1][1] for p in all_paths.values()) # 該論規劃路徑中的最大時間
                # 掃描所有機器人的最後狀態
                robot_last_state = {}
                for(pos, t), rid in self.__reservation_table.items():
                    if rid not in robot_last_state or t > robot_last_state[rid][1]:
                        robot_last_state[rid] = (pos, t)
                # 更新當前輪次的機器人
                for rid, path in all_paths.items():
                    robot_last_state[rid] = (path[-1][0], path[-1][1])

                # 延長所有機器人
                for rid, (pos, last_time) in robot_last_state.items():
                    for t in range(last_time+1, current_max_time+1):
                        if(pos, t) not in self.__reservation_table:
                            self.__reservation_table[(pos, t)] = rid
                # for rbid, rbpath in all_paths.items():
                #     finalpath_pos = rbpath[-1][0]
                #     finalpath_time = rbpath[-1][1]
                    # 如果finalpath_time比current_max_time小，則需要刪除這部分，再加
                    # 如果finalpath_time比current_max_time大，那麽finalpath_time就是current_max_time，
                    # 其他機器人需要補充從他們的finalpath_time到current_max_time

                # for rbid, rbpos in robot_real_end_pos.items():
                    # 延長到當前最大時間
                    # for t in range(finalpath_time + 1 , current_max_time + 1):
                    #     # if (final_pos, t) not in self.__reservation_table:
                    #     self.__reservation_table[(finalpath_pos, t)] = rbid

            # 所有機器人規劃 for循環結束
        # 更新全局時間偏移
        if all_paths:
            max_time = max(p[-1][1] for p in all_paths.values())
            self.__global_time_offset = max_time
            print(f"更新全局時間偏移至: {self.__global_time_offset}")

        # self._reserve_goal_indefinitely(all_paths)
        print(f"\n{'=' * 50}")
        print(f"✓ 所有机器人路径规划完成！")
        print(f"  最終預定表大小: {len(self.__reservation_table)}")
        print(f"{'=' * 50}")

        return all_paths

    # def plan_for_delivery(self,
    #          robot_start_id_dict: Dict[int, int],
    #          robot_goal_id_dict: Dict[int, Tuple[int, int]],
    #          priority_order: Optional[List[int]] = None) -> Dict[int, List[Tuple[int, int]]]:
    #     """
    #         CA*主函數：為所有機器人規劃協作路徑（支持取货-送货两段路径）
    #         參數:
    #            robot_start_id_dict: 機器人起始位置字典 {robot_id: start_position}
    #            robot_goal_id_dict: 機器人目標位置字典 {robot_id: (pickup_position, delivery_position)}
    #            priority_order: 可選的優先級順序列表，如果不提供則按robot_id排序
    #         返回:
    #         {robot_id: [(position, time), ...]}
    #         算法流程:
    #         1. 为每个机器人规划两段路径：
    #             - 第一段：start → pickup
    #             - 第二段：pickup → delivery
    #         2. 确保两段路径时间连续
    #         3. 整体处理冲突
    #     """
    #     # 保留 >= global_time_offset的預定表
    #     keys_to_remove = [k for k in self.__reservation_table.keys()
    #                       if k[1] < self.__global_time_offset]
    #     for k in keys_to_remove:
    #         del self.__reservation_table[k]
    #
    #     if set(robot_start_id_dict.keys()) != set(robot_goal_id_dict.keys()):
    #         print(f"Error: 起点和目标数量不匹配")
    #         return {}
    #
    #     all_paths: Dict[int, List[Tuple[int, int]]] = {}
    #
    #     if priority_order is None:
    #         robot_ids = sorted(robot_start_id_dict.keys())
    #     else:
    #         robot_ids = priority_order
    #         if set(robot_ids) != set(robot_start_id_dict.keys()):
    #             print(f"Error with priority order")
    #             return {}
    #     period_time = max(p[-1][1] for p in self.__reservation_table.keys())
    #     print(f"開始為 {len(robot_ids)} 個機器人規劃路徑（取货-送货模式）...")
    #     print(f"優先級順序: {robot_ids}")
    #     print(f"此階段的最早時間（上個階段的結尾時間）為：{period_time}")
    #
    #
    #     for idx, robot_id in enumerate(robot_ids):
    #         start = robot_start_id_dict[robot_id]
    #         pickup, delivery = robot_goal_id_dict[robot_id]
    #
    #         print(f"\n[{idx + 1}/{len(robot_ids)}] 規劃機器人 {robot_id}:")
    #         print(f"   路线: {start} → {pickup} (取货) → {delivery} (送货)")
    #
    #         # 重規劃循環
    #         max_replans = 10
    #         found_valid_path = False
    #         final_complete_path = None
    #
    #         for replan_count in range(max_replans):
    #             # 第一段：start 2 pickup
    #             path2pickup = self._plan_single_robot_with_offset(start, pickup, robot_id)
    #
    #             if path2pickup is None:
    #                 print(f"   錯誤：無法規劃到取货点 {pickup}")
    #                 return {}
    #
    #             pickup_time = path2pickup[-1][1]  # 到达取货点的时间
    #             if replan_count == 0:
    #                 print(f"      路徑長度：{len(path2pickup)} 步，到达時間 {pickup_time}")
    #             else:
    #                 print(f"      第 {replan_count} 次重規劃：長度 {len(path2pickup)} 步，時間 {pickup_time}")
    #
    #             # 檢查衝突
    #             has_conflict = False
    #             max_reservation_table_time = max(
    #                 t for (pos, t) in self.__reservation_table.keys()
    #             ) if self.__reservation_table else 0
    #
    #             if max_reservation_table_time > 0 and pickup_time >max_reservation_table_time:
    #                 occupied_positions = set()
    #                 for (pos, t) in self.__reservation_table.keys():
    #                     if t == max_reservation_table_time:
    #                         occupied_positions.add(pos)
    #
    #                 for pos, t in path2pickup:
    #                     if t > max_reservation_table_time and pos in occupied_positions:
    #                         print(f"     路徑在時間 {t} 經過位置 {pos}（其他機器人的終點）")
    #                         has_conflict = True
    #                         break
    #
    #             if not has_conflict:
    #                 found_valid_path = True
    #                 final_pickup_path = path2pickup
    #                 break
    #             # 有衝突，延長預定表
    #             print(f"    存在衝突，延長預定表並重新規劃...")
    #             for (pos, t) in list(self.__reservation_table.keys()):
    #                 if t == max_reservation_table_time:
    #                     occupying_robot = self.__reservation_table[(pos, t)]
    #                     for new_t in range(max_reservation_table_time+1, pickup_time+1):
    #                         if (pos, new_t) not in self.__reservation_table:
    #                             self.__reservation_table[(pos, new_t)] = occupying_robot
    #         if not found_valid_path:
    #             print(f"    到取货点的重新規劃次數超過限制！")
    #             return {}
    #
    #         # 預定到取货点的路徑
    #         self._reserve_path(final_pickup_path, robot_id)
    #         all_paths[robot_id] = final_pickup_path
    #         pickup_arrival_time = final_pickup_path[-1][1]
    #         print(f"    到取货点路徑已確認，到達時間 {pickup_arrival_time}")
    #         if all_paths:
    #             current_max_time = max(p[-1][1] for p in all_paths.values())
    #             for rbid, rbpath in all_paths.items():
    #                 final_pos = rbpath[-1][0]
    #                 final_time = rbpath[-1][1]
    #
    #                 # 延長到當前最大時間
    #                 for t in range(final_time + 1 , current_max_time + 1):
    #                     if (final_pos, t) not in self.__reservation_table:
    #                         self.__reservation_table[(final_pos, t)] = rbid
    #         # if
    #
    #
    #
    #         # 第二段：pickup 2 delivery
    #         # 从取货点开始的时间是 pickup_time
    #         # 需要临时修改 global_time_offset 来规划第二段
    #             original_offset = self.__global_time_offset
    #             self.__global_time_offset = pickup_time
    #
    #             path_to_delivery = self._plan_single_robot_with_offset(pickup, delivery, robot_id)
    #
    #             # 恢复原始偏移
    #             self.__global_time_offset = original_offset
    #
    #             if path_to_delivery is None:
    #                 print(f"   ❌ 錯誤：無法從取货点 {pickup} 規劃到送货点 {delivery}")
    #                 return {}
    #
    #             # ========== 合并两段路径 ==========
    #             # 注意：path_to_delivery 的第一个点是 pickup，与 path_to_pickup 的最后一个点重复
    #             complete_path = path2pickup + path_to_delivery[1:]  # ✅ 去掉重复点
    #
    #             current_path_end_time = complete_path[-1][1]
    #
    #             if replan_count == 0:
    #                 print(f"   第一段：{len(path2pickup)} 步，到达取货点时间 {pickup_time}")
    #                 print(f"   第二段：{len(path_to_delivery)} 步")
    #                 print(f"   完整路徑：{len(complete_path)} 步，完成時間 {current_path_end_time}")
    #             else:
    #                 print(
    #                     f"   第 {replan_count} 次重規劃：完整路徑 {len(complete_path)} 步，完成時間 {current_path_end_time}")
    #
    #             # ========== 檢查衝突 ==========
    #             has_conflict = False
    #             max_reservation_table_time = max(
    #                 t for (pos, t) in self.__reservation_table.keys()
    #             ) if self.__reservation_table else 0
    #
    #             if max_reservation_table_time > 0 and current_path_end_time > max_reservation_table_time:
    #                 # 找出在最大時間的所有終點位置
    #                 occupied_positions = set()
    #                 for (pos, t) in self.__reservation_table.keys():
    #                     if t == max_reservation_table_time:
    #                         occupied_positions.add(pos)
    #
    #                 # 檢查完整路徑是否經過這些終點位置
    #                 for pos, t in complete_path:
    #                     if t > max_reservation_table_time and pos in occupied_positions:
    #                         print(f"   ⚠️ 路徑在時間 {t} 經過位置 {pos}（其他機器人的終點）")
    #                         has_conflict = True
    #                         break
    #
    #             # ✅ 沒有衝突，找到有效路徑
    #             if not has_conflict:
    #                 found_valid_path = True
    #                 final_complete_path = complete_path
    #                 break
    #
    #             # ✅ 有衝突，延長預定表並繼續循環
    #             print(f"   🔄 延長預定表並準備重新規劃...")
    #             for (pos, t) in list(self.__reservation_table.keys()):
    #                 if t == max_reservation_table_time:
    #                     occupying_robot = self.__reservation_table[(pos, t)]
    #                     for new_t in range(max_reservation_table_time + 1, current_path_end_time + 1):
    #                         if (pos, new_t) not in self.__reservation_table:
    #                             self.__reservation_table[(pos, new_t)] = occupying_robot
    #
    #         # ✅ 檢查是否找到有效路徑
    #         if not found_valid_path:
    #             print(f"   ❌ 重新規劃次數超過限制 ({max_replans})！")
    #             return {}
    #
    #         # ✅ 預定最終完整路徑
    #         self._reserve_path(final_complete_path, robot_id)
    #         all_paths[robot_id] = final_complete_path
    #         print(f"   ✓ 機器人 {robot_id} 完整路徑已確認並預定")
    #
    #         # ✅ 延長所有已規劃機器人的終點到當前最大時間
    #         if all_paths:
    #             current_max_time = max(p[-1][1] for p in all_paths.values())
    #
    #             for rid, rpath in all_paths.items():
    #                 final_pos = rpath[-1][0]
    #                 final_time = rpath[-1][1]
    #
    #                 for t in range(final_time + 1, current_max_time + 1):
    #                     if (final_pos, t) not in self.__reservation_table:
    #                         self.__reservation_table[(final_pos, t)] = rid
    #
    #     # ✅ 更新全局時間偏移
    #     if all_paths:
    #         max_time = max(p[-1][1] for p in all_paths.values())
    #         self.__global_time_offset = max_time
    #         print(f"\n⏰ 更新全局時間偏移至: {self.__global_time_offset}")
    #
    #     print(f"\n{'=' * 50}")
    #     print(f"✓ 所有機器人路徑規劃完成！")
    #     print(f"  最終預定表大小: {len(self.__reservation_table)}")
    #     print(f"{'=' * 50}")
    #
    #     return all_paths




    def get_reservation_table(self) -> Dict[Tuple[int, int], int]:
        return self.__reservation_table.copy()

    def clear_reservation_table(self):
        self.__reservation_table.clear()


class CAstarPlanner(PathPlanner):
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
        # 一步 CA*
        if len(starts) != len(goals):
            print('Warning: Starts and Goals do not match!')
            return robot_paths

        first_robot = next(iter(starts))
        first_goal = starts[first_robot]
        path = self.plan_path(first_robot, first_goal)

        dynamic_obstacles = dict()
        for robot_id in starts:

            start_id = starts[robot_id]
            goal_id = goals[robot_id]
            if (list(starts).index(robot_id) == 0):
                path = self.plan_path(start_id, goal_id)

                dynamic_obstacles = {i: set(tuple(self._map.id2pos(index)[0], self._map.id2pos(index)[0])) for i, index in enumerate(path) }
                robot_paths[robot_id] = path
            else:
                path_id = []
                path = Planner.plan(self._map.id2pos(start_id), self._map.id2pos(goal_id),
                                    dynamic_obstacles)
                for x, y in path:
                    ind = self._node_pos2id((x,y))
                    path_id.append(ind)
                    robot_paths[robot_id] = path_id

        return robot_paths


    def plan_paths_from_CAstar(self,starts: Dict[int, int], goals: Dict[int, int],
                               priority_order: Optional[List[int]] = None,
                               idle_robots_positions: Optional[Dict[int, int]] = None) \
            -> Dict[int, List[int]]:
        """
        调用 CooperativeAStar 进行“协作式”规划，返回每个机器人的“节点ID序列”（按时间顺序）。
        参数：
            - starts/goals: {robot_id: node_id}
            - priority_order: 可选的优先级（不传默认按robot_id升序）
        """
        # 複用實例
        if not hasattr(self, '_ca_instance'):
            self._ca_instance = CooperativeAStar(self._map)
        # ca = CooperativeAStar(self._map)
        ca = self._ca_instance

        time_paths = ca.plan(starts, goals, priority_order)
        if not time_paths:
            return {}

        out: Dict[int, List[int]] = {}
        for rid, pt in time_paths.items():
            seq = [p for (p,_) in pt]
            if seq:
                comp = [seq[0]]
                for x in seq[1:]:
                    if x != comp[-1]:
                        comp.append(x)
                seq = comp
            out[rid] = seq
        return out


    def plan_paths_with_state(self, robot_start_id_dict: Dict[int, int], robot_goal_id_dict: Dict[int, int],
                              reservation_table: Dict[Tuple[int, int], int],
                              time_offset: int,
                              priority_order: Optional[List[int]] = None,
                              delivery: bool = False) \
            -> Tuple[Dict[int, List[int]], Dict[Tuple[int, int], int], int]:
        """
        带状态的路径规划
        参数:
            robot_start_id_dict/robot_goal_id_dict: 起点和终点
            robot_goal_id_dict會傳入兩種可能，通過delivery處理調用不同的規劃函數
                - Dict[robot_id: Tuple['goal', 'delivery_goal']]
                - Dict[robot_id: 'goal']
            reservation_table: 当前的预定表（会被复制，不修改原表）
            time_offset: 当前的全局时间偏移
            priority_order: 优先级
        返回:
            (路径字典, 更新后的预定表, 新的时间偏移)
        """
        if not delivery:  # - Dict[robot_id: 'goal']
            # 创建新的CooperativeAStar实例，傳入狀態
            ca = CooperativeAStar(self._map)

            # 傳入外部狀態
            ca.set_state(reservation_table.copy(), time_offset)

            # 執行規劃
            time_paths = ca.plan(robot_start_id_dict, robot_goal_id_dict, priority_order)
            if not time_paths:
                return {}, reservation_table, time_offset
            # 獲取更新后的狀態
            updated_table = ca.get_reservation_table()
            new_offset = ca.get_time_offset()

            # 转换为節點ID序列
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

        else:
            ca = CooperativeAStar(self._map)
            ca.set_state(reservation_table.copy(), time_offset)

            time_paths = ca.plan(robot_start_id_dict, robot_goal_id_dict, priority_order)



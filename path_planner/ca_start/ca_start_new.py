import networkx as nx
from typing import Tuple, List, Dict, Set, Optional
from heapq import heappush, heappop
from map_module.map import Map
from path_planner.path_planner import PathPlanner
from stastar.planner import Planner
from collections import defaultdict

class Node:
    """搜索節點"""
    def __init__(self, position: int, time: int, g_cost: float, h_cost: float,
                 parent: Optional['Node'] = None):
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


class CooperativeAStart(PathPlanner):

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

    def set_state(self, reservation_table: Dict[Tuple[int, int], int], time_offset: int):
        """設置狀態（用於外部調用）"""
        self.__spacetime_occupied = reservation_table.copy()
        self.__global_time_offset = time_offset

        # 從reservation_table重建previous_round_info  reservation_table默認是對齊了的
        self.__previous_round_info.clear()
        robot_last_state = {}
        reservation_table_max_time = max(
            (t for _, t in reservation_table.keys()), default=0
        )
        if reservation_table_max_time == 0:
            self.__previous_round_info = {}
        else:
            for (pos, t), id in reservation_table.items():
                if t == reservation_table_max_time:
                    robot_last_state[id] = (pos, t)
            self.__previous_round_info = robot_last_state

    def get_reservation_table(self) -> Dict[Tuple[int, int], int]:
        return self.__spacetime_occupied.copy()

    def get_time_offset(self) -> int:
        return self.__global_time_offset

    def clear_state(self) -> None:
        self.__spacetime_occupied.clear()
        self.__previous_round_info.clear()
        self.__current_round_paths.clear()
        self.__global_time_offset = 0

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
    def _is_valid_move(self, from_pos: int, to_pos: int, time: int, robot_id: int) -> bool:
        """檢查移動是否有效"""
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

    def _reconstruct_path(self, node: Node) -> List[Tuple[int, int]]:
        """從目標節點回溯構建路徑"""
        path =[]
        current = node
        while current is not None:
            path.append((current.position, current.time))
            current = current.parent
        return path[::-1]


    # 主要規劃功能函數

    # 對齊函數
    def _align_all_robots(self, current_round_path_max_time: int,
                          current_round_paths: Dict[int, List[Tuple[int, int]]]) -> None:
        if not current_round_paths:
            print("Current Round Paths Empty")
            return
        # 路徑在加入的時候，已經加到了__spacetime_occupied中，
        # 所以現在只需要延長到current_round_path_max_time
        for rid, path in current_round_paths.items():
            end_t = path[-1][1]
            end_p = path[-1][0]
            if end_t == current_round_path_max_time:
                continue
            for t in range(end_t+1, current_round_path_max_time+1):
                self.__spacetime_occupied[(end_p, t)] = rid
                # 雖然我覺得很沒有必要把path也延長吧？
                path.append((end_p,t))

    # 衝突檢查函數
    def _check_path_conflict(self, current_round_path_min_time: int,
                             current_round_path_max_time: int,
                             current_round_paths: Dict[int, List[Tuple[int, int]]],
                             sorted_by_time: bool = False) -> Optional[Dict]:
        """
        衝突有三類：
        1. vertex 頂點衝突：同一 (pos,t) 被 ≥2 台機器人占用
        2. edge 邊交換衝突：同一時刻 u->v 与 v->u 對向穿越
        3. goal 目標占位衝突：A 已在其重點 (gA, tA_end) 等待，B 在 t > tA_end 穿过 gA
        """
        # 每個機器人在每個時刻的位置，加速查詢
        timepos: Dict[int, Dict[int, int]] = defaultdict(dict) # {rid: {t: pos}}
        endpoints: Dict[int, Tuple[int, int]] = {}  # {rid: (end_pos, end_t)}
        for rid, path in current_round_paths.items():
            for pos, t in path:
                timepos[rid][t] = pos
            endpoints[rid] = path[-1]

        ### 頂點衝突在_is_valid_move裏面已經進行了避免
        # 頂點衝突 對每個t進行統計pos->[rids]
        # vertex_conflicts = []  # {'time': t, 'position': p, 'robots': [...]}
        # for t in range(current_round_path_min_time, current_round_path_max_time+1):
        #     occ = defaultdict(list)
        #     for rid, tp in timepos.items():
        #         if t in tp:
        #             occ[tp[t]].append(rid)
        #     for p, rlist in occ.items():
        #         if len(rlist) >= 2:
        #             vertex_conflicts.append({
        #                 'time': t, 'position': p, 'robots': sorted(rlist)
        #             })

        ### 邊交換衝突在_is_valid_move裏面已經進行了避免
        # 邊交換衝突  對每個t進行統計 u->v at t-1->t 同時存在 v->u at t-1->t
        # 使用 map 存 directed edge 使用
        # edge_conflicts = []  # {'time': t, 'edge': (u,v), 'robots': [r_small, r_big]}
        # for t in range(current_round_path_min_time+1, current_round_path_max_time+1):
        #     edge_use = defaultdict(list)  # (u,v) at time 由哪些rid使用
        #     for rid, tp in timepos.items():
        #         if (t-1) in tp and t in tp:
        #             u, v = tp[t-1], tp[t]
        #             if u!=v:
        #                 edge_use[(u,v)].append(rid)
        #     for (u,v), rlist_uv in edge_use.items():
        #         if (u,v) in edge_use:
        #             for r1 in rlist_uv:
        #                 for r2 in edge_use[(v,u)]:
        #                     if r1<r2:
        #                         edge_conflicts.append({
        #                             'time': t - 1, 'edge': (u, v), 'robots': [r1, r2]
        #                         })

        # 目標占位衝突 B 在 t>tA_end 穿过 A 的 (gA)
        goal_conflicts = []  # {'time': t, 'position': p, 'blocked_by': rid_fixed, 'robot': rid_pass}
        seen = set()  # 去重 (tt, gpos, rid_fix, rid)
        for rid_fix, (gpos, gtime) in endpoints.items():
            for rid, tp in timepos.items():
                if rid == rid_fix:
                    continue
                # 找 rid 在 gtime 之后是否路过 gpos
                for tt, p in tp.items():
                    if tt > gtime and p == gpos:
                        key = (tt, gpos, rid_fix, rid)
                        if key not in seen:
                            seen.add(key)
                            goal_conflicts.append({
                                'time': tt, 'position': gpos, 'blocked_by': rid_fix, 'robot': rid
                            })

        if sorted_by_time:
            # vertex_conflicts.sort(key = lambda x: x['time'])
            # edge_conflicts.sort(key = lambda  x: x['time'])
            goal_conflicts.sort(key = lambda x: x['time'])
            print(f"{'=' * 60} \n Goal_Conflicts:")
            for c in goal_conflicts:
                print(f"    [goal_conflicts] t={c['time']}, pos={c['position']}, "
                      f" blocked_by={c['blocked_by']}, robot={c['robot']}")

        # return {
        #     'vertex': vertex_conflicts,
        #     'edge': edge_conflicts,
        #     'goal_conflicts': goal_conflicts
        # }
        if not goal_conflicts:
            return None

        return {
            'goal_conflicts': goal_conflicts
        }

    # 處理goal conflicts函數
    def _resolve_goal_conflicts(self,
                                conflicts: Dict[str, List[Dict]],
                                robot_start_id_dict: Dict[int, int],
                                robot_goal_id_dict: Dict[int, int]
                                ) -> bool:
        """
        - conflicts {'goal_conflicts': [{'time': int, 'position': int,
                                         'blocked_by':int, 'robot':int}]}
        - previous_round_robots {robot_id: (end_position, end_time)}
        - prefer_replan_passer=True 时：固定 blocked_by，重規劃 robot（穿越者）
        - cmp_by_pathlen=True 时：对每个衝突比較兩者路徑長度，重規劃“路徑更短的一方”（代價小）

        返回True表示有重規劃發生（外层应再次跑一次检测-处置循环）
        """
        changed = False
        GoalConflicts_list = conflicts.get('goal_conflicts', [])

        # gaol_c = conflicts['goal_conflicts']
        # if not gaol_c:
        #     return False
        # 去重 取最早衝突時間的段落
        earliest_by_pair: Dict[Tuple[int, int], Dict] = {}
        for c in GoalConflicts_list:
            key = (c['blocked_by'], c['robot'])
            if key not in earliest_by_pair or c['time'] < earliest_by_pair[key]['time']:
                earliest_by_pair[key] = c

        to_replan: List[int] = []

        for (rid_fix, rid_pass), c in earliest_by_pair.items():
            # 因爲前面取了衝突最早的時間，衝突最早的時間就是時間步長長度較短的那個機器人，
            # 所以讓穿越者機器人進行重規劃
            rid_replan = rid_pass
            to_replan.append(rid_replan)

        # 對所有占用衝突進行一次重規劃
        for rid_replan in to_replan:
            old_path = self.__current_round_paths.pop(rid_replan, None)
            if old_path is not None:
                # 清理占用
                removed = 0
                for pos, t in old_path:
                    if self.__spacetime_occupied.get((pos,t)) == rid_replan:
                        del self.__spacetime_occupied[(pos, t)]
                        removed += 1
                print(f"  [resolve-goal] pop robot {rid_replan}, removed {removed} spacetime points")

            start = robot_start_id_dict[rid_replan]
            goal = robot_goal_id_dict[rid_replan]
            new_path = self._plan_single_robot_with_offset(start, goal, rid_replan)
            if new_path is None:
                print(f"  [resolve-goal] replan robot {rid_replan} failed.")
                continue
            print(f"  [resolve-goal] replan robot {rid_replan} success, and then updates new_path")
            # 更新 新路徑到__current_round_paths ， 並拿到新的最長時間步長
            self.__current_round_paths[rid_replan] = new_path
            # 占用回歸到__spacetime_occupied占用表
            for (pos, t) in new_path:
                self.__spacetime_occupied[(pos, t)] = rid_replan
            current_round_path_max_time = max(
                path[-1][1] for path in self.__current_round_paths.values())

            self._align_all_robots(current_round_path_max_time, self.__current_round_paths)
            # 應該要等所有conflicts處理完后，再進行衝突檢查吧？不然for循環裏面的item是無意義，
            # 因爲會送進去遞歸處理了，或者想辦法在這裏面增加判斷條件做break？
            changed = True
            print("  [resolve-goal] waiting for check if valid")
        return changed

    # 重規劃函數
    def _replan_robot_round(self, robot_id: int,
                            start: int, goal: int,
                            ) \
                            -> Optional[List[Tuple[int, int]]]:
        # 單次重規劃
        # 直接進行規劃，規劃完之後不進行檢查，誰調用重規劃就在哪個函數裏面檢查
        # 規劃完之後，只做時間步長的比較和對齊，若時間步長更長，則對齊__spacetime_occupied占用表之後重新進行規劃，
        # 并且更新current_round_path_max_time，直到時間步長不再增長爲止
        current_round_path_max_time = max(
            path[0][1] for path in self.__current_round_paths.values()
        )

        path = self._plan_single_robot_with_offset(start, goal, robot_id)
        if path is None:
            print(f"   重規劃機器人{robot_id} 無法找到路徑！")
            return None

        path_end_time = path[-1][1]
        replan_round = 0
        while path_end_time >= current_round_path_max_time:
            current_round_path_max_time = path_end_time
            # 延長除了robot id以外的占用
            self._align_all_robots(current_round_path_max_time, self.__current_round_paths)
            print(f"    重規劃路徑長度更新: {current_round_path_max_time},再次規劃")
            replan_path = self._plan_single_robot_with_offset(start, goal, robot_id)
            if path is None:
                print(f"    在重規劃第{replan_round+1}輪中，規劃失敗")
                return None
            path_end_time = replan_path[-1][1]
            replan_round += 1
        print(f"    共規劃第{replan_round}輪，完成機器人{robot_id}重規劃")

        return replan_path

    def _plan_single_robot_with_offset(self, start: int, goal: int, robot_id: int,
                                       max_time: int = 500) -> Optional[List[Tuple[int, int]]]:
        """為單個機器人構造路徑（考慮全局時間偏移）"""
        print(f"For single Robot {robot_id} Planning!")
        if start not in self.__G.nodes or goal not in self.__G.nodes:
            print(f" start {start} or goal {goal} is not in Map")
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
                print(f"find the goal {goal}")
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

    def plan(self, robot_start_id_dict: Dict[int, int],
                   robot_goal_id_dict: Dict[int, int],
                   priority_order: Optional[List[int]] = None,
                   is_RoundZero: bool = False) -> Dict[int, List[Tuple[int, int]]]:
        "CA*規劃函數"
        # 1. 前期准備
        if set(robot_start_id_dict.keys()) != set(robot_goal_id_dict.keys()):
            print("Error: 1")
            return {}
        if priority_order is None:
            robot_ids = sorted(robot_start_id_dict.keys())
        else:
            robot_ids = priority_order
            if set(robot_ids) != set(robot_goal_id_dict.keys()):
                print("Error: 2")
                return {}

        # 清空所有路徑再做規劃
        self.__current_round_paths.clear()
        # 當前機器人
        current_robot_set = set(robot_ids)
        # 非Round-0 需要獲取以前輪中的Robots的position
        if not is_RoundZero:
            if not self.__previous_round_info:
                print("Error: Reservation table don't exist!")
                return {}
            # {robot_id: (end_position, end_time)}
            previous_round_robots = self.__previous_round_info.copy()
        else:
            previous_round_robots = {}


        print(f"\n {'=' * 60} \n {'=' * 60} \n 開始新一輪規劃 \n 當前機器人: {robot_ids}")
        if previous_round_robots:
            print(f"上一輪機器人: {list(previous_round_robots.keys())}")
            for rid, (pos, t) in previous_round_robots.items():
                print(f"  機器人 {rid}: 位置 {pos}, 時間 {t}")
        print(f"全局時間偏移: {self.__global_time_offset} \n {'=' * 60} \n {'=' * 60}")


        # 2. 單個機器人的規劃
        for idx, robot_id in enumerate(robot_ids):
            start = robot_start_id_dict[robot_id]
            goal = robot_goal_id_dict[robot_id]
            print(f"\n[{idx + 1}/{len(robot_ids)}] 規劃機器人 {robot_id}: {start} -> {goal}")

            path = self._plan_single_robot_with_offset(start, goal, robot_id)
            if path is None:
                print("   無法找到路徑！")
                return None
            print(f"   初始路徑: 長度 {len(path)}, 結束時間 {path[-1][1]}")

            # 非Round0且是該論第一個機器人，要考慮上一輪的機器人最終位置__previous_round_info
            # 已經寫好在previous_round_robots裏面  {robot_id: (end_position, end_time)}
            # 需要考慮嗎？要不直接給，最後這一輪規劃完，做檢查，有衝突再領出來重新規劃！
            # if current_round_path_max_time == 0 and not is_RoundZero:

            # 加入到__spacetime_occupied 同時更新 __current_round_paths
            # 先規劃完，再對所有進行檢查，檢查完后有衝突則重新規劃
            self.__current_round_paths[robot_id] = path
            for (pos, t) in path:
                self.__spacetime_occupied[(pos, t)] = robot_id
        print(f" \n '=' * 60 \n 本輪機器人初次規劃已經完成，共有{len(self.__current_round_paths)} \n 下面進行衝突檢查 \n '=' * 60")

        # 先進行對齊，便於檢查後完成重規劃
        current_round_path_min_time = min(
                path[0][1] for path in self.__current_round_paths.values())
        current_round_path_max_time = max(
                path[0][1] for path in self.__current_round_paths.values())
        # _check_path_conflict比較的是paths，而不是對齊後的占用表，好像沒有必要在這裏對齊？
        self._align_all_robots(current_round_path_max_time, self.__current_round_paths)

        # 3.衝突檢查 與重規劃
        conflict = self._check_path_conflict(current_round_path_min_time, current_round_path_max_time,
                                             self.__current_round_paths, sorted_by_time=True)
        while conflict and conflict.get('goal_conflicts'):
            # _resolve_goal_conflicts函數裏面也有對齊，不過是在產生新的路徑后再
            resolver_flag = self._resolve_goal_conflicts(conflict, robot_start_id_dict, robot_goal_id_dict)
            if not resolver_flag:
                print(f"Break by resolver: {resolver_flag}")
                return {}

            current_round_path_min_time = min(
                path[0][1] for path in self.__current_round_paths.values())
            current_round_path_max_time = max(
                path[-1][1] for path in self.__current_round_paths.values())
            conflict = self._check_path_conflict(current_round_path_min_time, current_round_path_max_time,
                                                 self.__current_round_paths, sorted_by_time=True)

        return self.__current_round_paths




"""
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
"""
class CAstarPlanner(PathPlanner):
    def __init__(self, map_input: Map):
        super().__init__(map_input)
        self.__G = map_input.G

    def plan_paths_with_state(self,
                              robot_start_id_dict: Dict[int, int],
                              robot_goal_id_dict: Dict[int, int],
                              reservation_table: Dict[Tuple[int, int], int],
                              time_offset: int,
                              priority_order: Optional[List[int]] = None,
                              is_RoundZero: bool = False,
                              delivery: bool = False) \
            -> Tuple[Dict[int, List[int]], Dict[Tuple[int, int], int], int]:
        """帶狀態的路徑規劃"""
        ca = CooperativeAStart(self._map)
        ca.set_state(
            reservation_table = reservation_table.copy(),
            time_offset = time_offset)

        time_paths = ca.plan(robot_start_id_dict, robot_goal_id_dict,
                             is_RoundZero = is_RoundZero)
        if not time_paths:
            return {}, reservation_table, time_offset

        # 更新后的狀態
        updated_table = ca.get_reservation_table()
        new_timeoffset = ca.get_time_offset()

        # 轉換為節點ID序列
        out: Dict[int, List[int]] = {}
        for rid, pt in time_paths.items():
            out[rid] = [pos for (pos, _t) in pt]

        return out, updated_table, new_timeoffset

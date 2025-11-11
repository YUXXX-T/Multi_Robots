import numpy as np
from map_module.map import Map
import math
from scipy.optimize import linear_sum_assignment

class KM:
    def __init__(self,map_path,robots,tasks):
        map_path = 'random-32-32-10.json'
        self.demo_map = Map(map_path)
        self.matrix = None
        self.max_weight = 0
        self.row, self.col = 0, 0  
        self.size = 0   # 方阵大小
        self.lx = None  # 左侧权值
        self.ly = None  # 右侧权值
        self.match = None   # 匹配结果
        self.slack = None   # 边权和顶标最小的差值
        self.visx = None    # 左侧是否加入增广路
        self.visy = None    # 右侧是否加入增广路
        self.robots = robots
        self.tasks = tasks

    def cal_cost_matrix(self):
        # 歐式距離
        num_ = len(self.tasks)
        cost_matrix = np.zeros((num_,num_))
        for i in range(num_):
            robot_index = self.robots[i].node_id()
            robot_positon = self.demo_map.id2pos(robot_index)
            for j in range(num_):
                task_index = self.tasks[j]['goal']
                task_positon = self.demo_map.id2pos(task_index)

                cost = math.sqrt((robot_positon[0]-task_positon[0])*(robot_positon[0]-task_positon[0])+(robot_positon[1]-task_positon[1])*(robot_positon[1]-task_positon[1]))
                cost_matrix[i][j] = cost
        # pass
        return cost_matrix
    def hungarian_algorithm_mannual(self, cost_matrix):
        n=len(cost_matrix)

        matrix = cost_matrix.copy().astype(np.float64)

        u = np.zeros(n+1, dtype=np.float64) # 行
        v = np.zeros(n+1, dtype=np.float64) # 列
        p = np.zeros(n+1, dtype=int)  # p[j] = i 表示：任务 j 当前分给了机器人 i（i=0 表示任务 j 没分配）。
        way = np.zeros(n+1, dtype=int) # 记录调整分配的路径

        for i in range(1, n + 1):
            p[0] = i
            minv = np.full(n + 1, np.inf)
            used = np.zeros(n + 1, dtype=bool)
            j0 = 0
            while True:
                used[j0] = True
                i0 = p[j0]
                delta = np.inf
                j1 = 0
                # 寻找最小差值并更新路径
                for j in range(1, n + 1):
                    if not used[j]:
                        cur = matrix[i0 - 1, j - 1] - u[i0] - v[j]
                        if cur < minv[j]:
                            minv[j] = cur
                            way[j] = j0
                        if minv[j] < delta:
                            delta = minv[j]
                            j1 = j
                # 更新标签
                for j in range(n + 1):
                    if used[j]:
                        u[p[j]] += delta
                        v[j] -= delta
                    else:
                        minv[j] -= delta
                j0 = j1
                if p[j0] == 0:
                    break
            # 调整匹配
            while True:
                j1 = way[j0]
                p[j0] = p[j1]
                j0 = j1
                if j0 == 0:
                    break

            # 生成与 linear_sum_assignment 一致的输出格式
            # p[j] 是行索引（1-based），j 是列索引（1-based），需转换为0-based
            # row_ind[j] 是分配给第 j 个任务（列 j）的机器人/行索引 i  col_ind = [0, 1, 2, ..., n-1]（按列/任务的自然顺序）
        col_ind = np.arange(n, dtype=int)  # 列索引范围0~n-1
        row_ind = np.array([p[j + 1] - 1 for j in col_ind], dtype=int)  # 转换为0-based行索引

        return row_ind, col_ind


    def compute(self):
        cost_matrix = self.cal_cost_matrix()
        sorted_rows, sorted_cols = [], []
        # TODO: 根据代价矩阵计算最小匹配

        # row_ind, col_ind  = linear_sum_assignment(cost_matrix)
        row_ind, col_ind = self.hungarian_algorithm_mannual(cost_matrix)
        # return sorted_rows, sorted_cols
        return row_ind, col_ind

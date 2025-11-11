import os
import json
import random
from tqdm import tqdm

from map_module.map import Map
from env_module.board import Board


if __name__ == '__main__':
    map_name = 'map_rect'
    map_path = 'map_rect.json'.format(map_name)
    save_dir = 'D:\\Phanes\\senarios\\zuoye\\{}'.format(map_name)
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    demo_map = Map(map_path)
    robot_num = 10
    board = Board(demo_map, robot_num)

    # region Classic Demo
    # robot_starts = {
    #     0: (2, 5),
    #     1: (5, 3)
    # }
    # robot_ends = {
    #     0: (2, 0),
    #     1: (0, 5)
    # }
    # for robot_id in robot_starts:
    #     robot_starts[robot_id] = demo_map.pos2id(robot_starts[robot_id])
    # for robot_id in robot_ends:
    #     robot_ends[robot_id] = demo_map.pos2id(robot_ends[robot_id])
    # endregion

    for senario_iter in tqdm(range(10)):
        node_id_ls = demo_map.nodes_by_type('normal')
        selected_node_id_ls = random.sample(node_id_ls, robot_num * 2)
        robot_starts = dict()
        robot_ends = dict()
        for robot_id in range(robot_num):
            robot_starts[robot_id] = selected_node_id_ls[robot_id * 2]
            robot_ends[robot_id] = selected_node_id_ls[robot_id * 2 + 1]

        data = dict()
        data['RobotNum'] = robot_num
        data['Tasks'] = dict()
        for robot_id in robot_starts:
            data['Tasks'][robot_id] = {
                'start': robot_starts[robot_id],
                'goal': robot_ends[robot_id]
            }

        senario_name = 'case-{}'.format(senario_iter + 1)
        senario_dir = '{}/{}'.format(save_dir, senario_name)
        if not os.path.exists(senario_dir):
            os.makedirs(senario_dir)
        board.show_tasks(robot_starts, robot_ends,
                         filename='{}/{}'.format(senario_dir, 'render'), plt_show=False)
        with open('{}/data.json'.format(senario_dir), 'w') as f:
            json.dump(data, f, indent=4, ensure_ascii=False)

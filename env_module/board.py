import random
import numpy as np
from typing import Dict, List
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle,Rectangle
from matplotlib.collections import LineCollection
from matplotlib.pyplot import text
from map_module.map import Map
from env_module.robot import Robot


def generate_random_color():
    """
    生成随机颜色的RGB值
    """
    r = random.randint(0, 255)
    g = random.randint(0, 255)
    b = random.randint(0, 255)
    return r / 255, g / 255, b / 255  


def generate_random_colors(num_colors):
    """
    生成指定数量的随机颜色列表
    """
    colors = [generate_random_color() for _ in range(num_colors)]
    return colors


class Board:
    def __init__(self, map_input: Map, robot_num=0):
        self.map = map_input
        self.nodes = map_input.nodes()
        self.graph = map_input.G

        self.fig_size = map_input.fig_size
        self.node_plot = map_input.node_plot
        self.edge_plot = map_input.edge_plot
        self.robot_colors = [(1, 0.5, 0)] * robot_num 

    def __show_map(self, show_node_id=True, show_roadmap=True, show_path=False, paths: Dict[int, List] = None,
                   file_name: str = None, plt_show=True, show_robot=False, robots: Dict[int, Robot] = None,
                   starts: Dict[int, int] = None, goals: Dict[int, int] = None, show_task=False):
        show_node_id = show_node_id
        show_roadmap = show_roadmap
        show_path = show_path and paths is not None
        show_robot = show_robot and robots is not None
        show_task = show_task and starts is not None and goals is not None

        self.__plt_graph(show_node_id, show_roadmap)

        if show_path:
            self.__plt_paths(paths)
        if show_robot:
            self.__plt_robots(robots)
        if show_task:
            self.__plt_tasks(starts, goals)

        self.__plt_end(file_name, plt_show)

    def __plt_graph(self, show_node_id, show_roadmap, plt_scatter=True):
        node_id2node = self.nodes
        graph = self.graph
        fig_size = self.fig_size
        node_plot = self.node_plot
        edge_plot = self.edge_plot
        show_node_id = show_node_id
        show_roadmap = show_roadmap
        node_ids = [node['id'] for node in node_id2node.values()]
        x_ls = [node['pos'][0] for node in node_id2node.values()]
        y_ls = [node['pos'][1] for node in node_id2node.values()]

        fig_width = fig_size['width']
        fig_height = fig_size['height']
        plt.figure(figsize=(fig_width, fig_height))
        if show_node_id:
            for i in range(len(x_ls)):
                plt.text(x_ls[i], y_ls[i], str(node_ids[i]), color='w',
                         ha='center', va='center', fontsize=10, zorder=20)
        if plt_scatter:
            for node_type in node_plot:
                x_ls, y_ls = [], []
                for node in node_id2node.values():
                    if node['type'] == node_type:
                        x_ls.append(node['pos'][0])
                        y_ls.append(node['pos'][1])
                # print("=="*25, f" color: {node_plot[node_type]['color']}", "=="*25)
                plt.scatter(x_ls, y_ls, marker=node_plot[node_type]['marker'], color=node_plot[node_type]['color'],
                            s=node_plot[node_type]['s'], zorder=node_plot[node_type]['zorder'])

        if show_roadmap:
            for edge in graph.edges():
                init_node_id, end_node_id = edge
                init_x, init_y = node_id2node[init_node_id]['pos']
                end_x, end_y = node_id2node[end_node_id]['pos']

                plt.plot([init_x, end_x], [init_y, end_y], color=edge_plot['color'],
                         linestyle=edge_plot['linestyle'], linewidth=edge_plot['linewidth'],
                         zorder=edge_plot['zorder'])
    
    def __plt_paths(self, paths: Dict[int, List[int]]):
        node_id2node = self.nodes
        for robot_id, path in paths.items():
            if path:  
                current_pos = node_id2node[path[0]]['pos']
                goal_pos = node_id2node[path[-1]]['pos']
                plt.plot([current_pos[0], goal_pos[0]], [current_pos[1], goal_pos[1]],
                        color=self.robot_colors[robot_id], linewidth=1, linestyle='-',zorder=19) 
                plt.scatter([goal_pos[0]], [goal_pos[1]], color=self.robot_colors[robot_id], zorder=21, marker='*',s=500)

    
    def __plt_robot(self, robot: Robot):
        robot_pos = np.array(robot.pos())
        robot_dir = robot.dir()

        car_width = 0.6
        car_length = 0.6

        angle = np.radians(-robot_dir)
        robot_color = self.robot_colors[robot.id()] if robot.state()=="IDLE" else (0.5, 0.0, 0.3)
        rect = Rectangle(
            xy=(robot_pos[0] - car_width / 2, robot_pos[1] - car_length / 2),
            width=car_width,
            height=car_length,
            angle=angle,  
            edgecolor='black',
            facecolor = robot_color
        )# facecolor = self.robot_colors[robot.id()]

        text_pos = robot_pos
        plt.text(text_pos[0], text_pos[1], str(robot.id()), color='black', ha='center', va='center', fontsize=10, zorder=30)

        plt.gca().add_patch(rect)

        return rect

    def __plt_robots(self, robot_poses: Dict[int, Robot]):
        for _, robot in robot_poses.items():
            robot_patch = self.__plt_robot(robot)
            robot_patch.set_zorder(25)
            plt.gca().add_patch(robot_patch)
    
    def __plt_tasks(self, starts: Dict[int, int], goals: Dict[int, int]):
        for robot_id in starts:
            start_node_id = starts[robot_id]
            end_node_id = goals[robot_id]
            start_node = self.nodes[start_node_id]
            end_node = self.nodes[end_node_id]
            start_pos = start_node['pos']
            end_pos = end_node['pos']
            color = self.robot_colors[robot_id]
            plt.arrow(start_pos[0], start_pos[1], end_pos[0] - start_pos[0], end_pos[1] - start_pos[1],
                      head_width=0.25, head_length=0.5, fc=color, ec=color, zorder=20, length_includes_head=True,
                      linewidth=5)

    def __plt_end(self,file_name: str = None, plt_show=True):
        
        plt.tight_layout()
        plt.xlim(-0.5, self.fig_size['width']*2)
        plt.ylim(-0.5, self.fig_size['height']*2)
        if file_name is not None:
            file_name = '{}.png'.format(file_name)
            plt.savefig(file_name)
        if plt_show:
            plt.show()
        plt.close()

    def show_map(self, show_node_id=False):
        self.__show_map(show_node_id=show_node_id, show_roadmap=True)

    def show_paths(self, paths: Dict[int, List], filename: str = None, plt_show=True):
        self.__show_map(show_path=True, paths=paths, file_name=filename, plt_show=plt_show)

    def show_robots(self, robots: Dict[int, Robot], paths: Dict[int, List], filename: str = None, plt_show=True):
        self.__show_map(show_path=True, show_robot=True, robots=robots, paths=paths,
                        file_name=filename, plt_show=plt_show)

    def show_tasks(self, starts: Dict[int, int], goals: Dict[int, int], filename: str = None, plt_show=True):
        self.__show_map(show_task=True, starts=starts, goals=goals, file_name=filename, plt_show=plt_show)


def show_map(map_input: Map, show_node_id=False):
    board = Board(map_input)
    board.show_map(show_node_id=show_node_id)


if __name__ == '__main__':
    map_path = 'C:\\Users\\34995\\Desktop\\Phanes\\maps\\CaDemo\\map.json'
    demo_map = Map(map_path)
    show_map(demo_map, show_node_id=False)

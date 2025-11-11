import json
import networkx as nx

from typing import Tuple


class Map:
    def __init__(self, map_path):
        self.data = json.load(open(map_path, 'r', encoding='utf-8'))
        self.fig_size, self.node_plot, self.edge_plot = (
            self.data['FigSize'], self.data['NodePlot'], self.data['EdgePlot'])
        self.__nodes = {int(k): v for k, v in self.data['Nodes'].items()}

        self.__node_id2pos, self.__node_pos2id = dict(), dict()
        for node_id, attributes in self.__nodes.items():
            attributes['pos'] = tuple(attributes['pos'])
            self.__node_id2pos[node_id] = attributes['pos']
            self.__node_pos2id[attributes['pos']] = node_id

        self.G = nx.Graph()
        self.__init_graph()

    def __init_graph(self):
        for node_id, attributes in self.__nodes.items():
            self.G.add_node(node_id, **attributes)
        for node_id, attributes in self.__nodes.items():
            for neighbor_id in attributes['neighbors']:
                self.G.add_edge(node_id, neighbor_id)

    def nodes(self):
        return self.__nodes

    def pos2id(self, pos: Tuple[int, int]):
        return self.__node_pos2id[pos]

    def id2pos(self, node_id: int):
        return self.__node_id2pos[node_id]
    
    def get_id2pos(self):
        return self.__node_id2pos

    def node_str(self, node_id):
        node = self.__nodes[node_id]
        return str(node).replace('{', '(').replace('}', ')')

    def __str__(self):
        map_str = ('The map is in the form of a topological map and is thus composed of and '
                   'described by node information, with the information of each node as follows:\n\n')
        for node_id in self.__nodes:
            map_str += self.node_str(node_id) + '\n'
        # map_str += '\n' + '#' * 100
        return map_str

    def nodes_by_type(self, node_type):
        return [k for k, v in self.__nodes.items() if v['type'] == node_type]
    
    def get_node(self, id: int):
        return self.nodes()[id]
import os
import json
import numpy as np
import matplotlib.pyplot as plt


def convert_type(node_str: str) -> int:
    if node_str in ['T', '@']:
        return 1
    elif node_str in ['.']:
        return 0
    else:
        return -1


MAP_NAME = 'random-32-32-10'
MAP_DIR_NAME = 'MAPF'
SAVE_PATH = 'D:\\Phanes\\maps\\' + MAP_DIR_NAME
INPUT_DIR = 'D:\\IRMV\\多机器人\\地图\\mapf-map'
INPUT_PATH = '{}\\{}.map'.format(INPUT_DIR, MAP_NAME)

node_types = ['normal', 'blocked']
data = {'FigSize': {}, 'NodePlot': {}, 'EdgePlot': {}, 'Nodes': {}}
for node_type in node_types:
    data['NodePlot'][node_type] = {}

with open(INPUT_PATH, 'r') as f:
    lines = f.readlines()
    height = int(lines[1].split()[1].split('\n')[0])
    width = int(lines[2].split()[1].split('\n')[0])
    map_grid = np.zeros((height, width))

    map_lines = lines[4:]
    for i in range(height):
        map_line = map_lines[i]
        for j in range(width):
            map_grid[i][j] = convert_type(map_line[j])

    node_id = 0
    nodes = dict()
    node_pos2id = dict()
    for x in range(width):
        for y in range(height):
            nodes[node_id] = dict()
            nodes[node_id]['id'] = node_id
            nodes[node_id]['pos'] = (x, y)
            nodes[node_id]['type'] = 'normal' if map_grid[y][x] == 0 else 'blocked'
            node_pos2id[(x, y)] = node_id
            node_id += 1

    for node_id in nodes:
        nodes[node_id]['neighbors'] = list()
        if nodes[node_id]['type'] == 'blocked':
            continue
        for x, y in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            neighbor_x = nodes[node_id]['pos'][0] + x
            neighbor_y = nodes[node_id]['pos'][1] + y
            neighbor_id = node_pos2id.get((neighbor_x, neighbor_y))
            if neighbor_id is not None and nodes[neighbor_id]['type'] != 'blocked':
                nodes[node_id]['neighbors'].append(neighbor_id)
    data['Nodes'] = nodes

    plt.figure(figsize=(width, height))
    s_general = 1000
    zorder_general = 10
    marker_general = 's'
    for node_type in data['NodePlot']:
        if node_type == 'normal':
            marker = marker_general
            color = 'white'
            s = s_general
            zorder = zorder_general
        elif node_type == 'blocked':
            marker = marker_general
            color = 'black'
            s = s_general
            zorder = zorder_general
        else:
            continue

        x_ls = [nodes[node_id]['pos'][0] for node_id in nodes if nodes[node_id]['type'] == node_type]
        y_ls = [nodes[node_id]['pos'][1] for node_id in nodes if nodes[node_id]['type'] == node_type]
        plt.scatter(x_ls, y_ls, marker=marker, color=color, s=s, zorder=zorder)
        data['NodePlot'][node_type]['marker'] = marker
        data['NodePlot'][node_type]['color'] = color
        data['NodePlot'][node_type]['s'] = s
        data['NodePlot'][node_type]['zorder'] = zorder

    color = 'white'
    linestyle = '-'
    linewidth = 1
    zorder = 1
    for node_id in nodes:
        # plt.text(nodes[node_id]['pos'][0], nodes[node_id]['pos'][1], str(node_id),
        #          ha='center', va='center', c='white', zorder=15)
        # for neighbor_id in nodes[node_id]['neighbors']:
        #     plt.plot([nodes[node_id]['pos'][0], nodes[neighbor_id]['pos'][0]],
        #              [nodes[node_id]['pos'][1], nodes[neighbor_id]['pos'][1]],
        #              color=color, linestyle=linestyle, linewidth=linewidth, zorder=zorder)
        if data['EdgePlot'] == {}:
            data['EdgePlot']['color'] = color
            data['EdgePlot']['linestyle'] = linestyle
            data['EdgePlot']['linewidth'] = linewidth
            data['EdgePlot']['zorder'] = zorder

    plt.title('Node Map')
    plt.xlabel('X-coordinate')
    plt.ylabel('Y-coordinate')
    # plt.grid(True)
    plt.tight_layout()
    plt.axis('equal')

    fig = plt.gcf()
    fig_size = fig.get_size_inches()
    data['FigSize']['width'] = fig_size[0]
    data['FigSize']['height'] = fig_size[1]
    print('FigSize:', data['FigSize'])

    plt.show()

    ask_user = input('Do you want to continue? (y/n) ')
    if ask_user == 'y':
        if not os.path.exists(SAVE_PATH):
            os.makedirs(SAVE_PATH)
        json.dump(data, open('{}/{}.json'.format(SAVE_PATH, MAP_NAME), 'w', encoding='utf-8'),
                  indent=4, ensure_ascii=False)
    else:
        exit()

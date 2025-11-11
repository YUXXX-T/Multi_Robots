import os
import json
import matplotlib.pyplot as plt

MAP_NAME = 'CaDemo'
SAVE_PATH = 'C:\\Users\\34995\\Desktop\\Phanes\\maps\\' + MAP_NAME
node_types = ['normal', 'blocked']
blocked_node_id_ls = [6, 9, 13, 16, 19, 26, 27, 29, 30]
data = {'FigSize': {}, 'NodePlot': {}, 'EdgePlot': {}, 'Nodes': {}}
for node_type in node_types:
    data['NodePlot'][node_type] = {}

node_id = 0
nodes = dict()
node_pos2id = dict()
for x in range(6):
    for y in range(6):
        nodes[node_id] = dict()
        nodes[node_id]['id'] = node_id
        nodes[node_id]['pos'] = (x, y)
        nodes[node_id]['type'] = 'normal' if node_id not in blocked_node_id_ls else 'blocked'
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

plt.figure(figsize=(6, 6))
s_general = 500
zorder_general = 10
marker_general = 's'
for node_type in data['NodePlot']:
    if node_type == 'normal':
        marker = marker_general
        color = '#5B9BD5'
        s = s_general
        zorder = zorder_general
    elif node_type == 'blocked':
        marker = marker_general
        color = 'red'
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

color = 'black'
linestyle = '-'
linewidth = 1
zorder = 1
for node_id in nodes:
    plt.text(nodes[node_id]['pos'][0], nodes[node_id]['pos'][1], str(node_id),
             ha='center', va='center', c='white', zorder=15)
    for neighbor_id in nodes[node_id]['neighbors']:
        plt.plot([nodes[node_id]['pos'][0], nodes[neighbor_id]['pos'][0]],
                 [nodes[node_id]['pos'][1], nodes[neighbor_id]['pos'][1]],
                 color=color, linestyle=linestyle, linewidth=linewidth, zorder=zorder)
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
    json.dump(data, open('{}/map.json'.format(SAVE_PATH), 'w', encoding='utf-8'),
              indent=4, ensure_ascii=False)
else:
    exit()

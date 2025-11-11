class Node:
    def __init__(self, node_id: int, cost: int, heuristic: int, time=None, parent=None):
        self.node_id = node_id
        self.cost = cost
        self.heuristic = heuristic
        self.time = time
        self.parent = parent

    def __lt__(self, other):
        return self.total_cost < other.total_cost

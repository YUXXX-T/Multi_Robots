import os
import time
from env_module.env import Env


if __name__ == '__main__':
    map_path = 'random-32-32-10.json'

    log_dir = 'log\\{}'.format(
        time.strftime("%Y-%m-%d %H-%M-%S", time.localtime()))
    solver = 'ca_star'  # planning solvers: such as astar ...
    task_generate_seed = None
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    env = Env(map_path, slover=solver, seed=task_generate_seed)
    env.run(log_dir)




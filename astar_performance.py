import numpy as np
from planning_utils import a_star, heuristic, create_grid
from timeit import default_timer as timer


map_data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
grid, heightmap, north_offset, east_offset = create_grid(map_data, 5, 5)

# Trivial goal (first mission waypoint in LOCATIONS.md)
grid_start = (315, 445)
grid_goal = (306, 316)

start = timer()
path, _ = a_star(grid, heuristic, grid_start, grid_goal)
print(f'Path planning to goal completed in {timer() - start:.3} s.')

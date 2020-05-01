from enum import Enum
from queue import PriorityQueue
from typing import Tuple, Callable, Dict, List
import numpy as np


Position = Tuple[int, int]
HeuristicFunction = Callable[[Position, Position], float]

SQRT2 = np.sqrt(2)


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))
    height_map = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]

        obstacle = [
            int(np.clip(north - d_north - safety_distance - north_min, 0, north_size - 1)),
            int(np.clip(north + d_north + safety_distance - north_min, 0, north_size - 1)),
            int(np.clip(east - d_east - safety_distance - east_min, 0, east_size - 1)),
            int(np.clip(east + d_east + safety_distance - east_min, 0, east_size - 1)),
        ]
        height_map[obstacle[0]:obstacle[1] + 1, obstacle[2]:obstacle[3] + 1] = alt + d_alt

        if alt + d_alt + safety_distance > drone_altitude:
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, height_map, int(north_min), int(east_min)


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1.0)
    EAST = (0, 1, 1.0)
    NORTH = (-1, 0, 1.0)
    SOUTH = (1, 0, 1.0)

    NORTH_WEST = (-1, -1, SQRT2)
    NORTH_EAST = (-1, 1, SQRT2)
    SOUTH_WEST = (1, -1, SQRT2)
    SOUTH_EAST = (1, 1, SQRT2)

    @property
    def cost(self) -> float:
        return self.value[2]

    @property
    def delta(self) -> Tuple[int, int]:
        return self.value[0], self.value[1]


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    actions = []  # type: List[Action]
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    north_clear = x > 0 and grid[x - 1, y] == 0
    south_clear = x + 1 < n and grid[x + 1, y] == 0
    west_clear = y > 0 and grid[x, y - 1] == 0
    east_clear = y + 1 < m and grid[x, y + 1] == 0

    if north_clear:
        actions.append(Action.NORTH)
        if east_clear:
            actions.append(Action.NORTH_EAST)
        if west_clear:
            actions.append(Action.NORTH_WEST)

    if south_clear:
        actions.append(Action.SOUTH)
        if east_clear:
            actions.append(Action.SOUTH_EAST)
        if west_clear:
            actions.append(Action.SOUTH_WEST)

    if west_clear:
        actions.append(Action.WEST)
    if east_clear:
        actions.append(Action.EAST)

    return actions


def a_star(grid: np.ndarray, h: HeuristicFunction, start: Position, goal: Position)\
        -> Tuple[List[Position], float]:
    assert 0 <= goal[0] < grid.shape[0]
    assert 0 <= goal[1] < grid.shape[1]

    assert grid[start[0], start[1]] == 0, "Start is in invalid position."
    assert grid[goal[0], goal[1]] == 0, "Goal is in invalid position."

    path = []  # type: List[Position]
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}  # type: Dict[Position, Tuple[float, Position, Action]]
    found = False

    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:
            current_cost = branch[current_node][0]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
    return path[::-1], path_cost


def heuristic(position: Position, goal_position: Position) -> float:
    return np.linalg.norm(np.array(position) - np.array(goal_position))

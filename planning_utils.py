from enum import Enum
from typing import Tuple, Callable, Dict, List
import numpy as np
import heapq


GridPosition = Tuple[int, int]
HeuristicFunction = Callable[[GridPosition, GridPosition], float]

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

    def is_valid(self, grid: np.ndarray, current_node: GridPosition) -> bool:
        """
        Determines whether the specified action is valid, given the the current node and the grid.
        :param grid: The grid to verify the action for.
        :param current_node: The current node to which to apply the action.
        :return: True if the action is admissible; False otherwise.
        """
        n, m = grid.shape[0] - 1, grid.shape[1] - 1
        x = current_node[0] + self.delta[0]
        y = current_node[1] + self.delta[1]
        return (0 <= x < n) and (0 <= y < m) and (grid[x, y] == 0)


def same_node(a: GridPosition, b: GridPosition) -> bool:
    return a == b


def a_star(grid: np.ndarray, h: HeuristicFunction, start: GridPosition, goal: GridPosition)\
        -> Tuple[List[GridPosition], float]:
    assert 0 <= goal[0] < grid.shape[0]
    assert 0 <= goal[1] < grid.shape[1]

    assert grid[start[0], start[1]] == 0, "Start is in invalid position."
    assert grid[goal[0], goal[1]] == 0, "Goal is in invalid position."

    path = []  # type: List[GridPosition]
    path_cost = 0
    heap = []  # type: List[Tuple[float, GridPosition]] # Priority queue implemented as a heap
    heapq.heappush(heap, (0., start))
    visited = set(start)

    # noinspection PyTypeChecker
    possible_actions = list(Action)  # type: List[Action]

    branch = {}  # type: Dict[GridPosition, Tuple[float, GridPosition, Action]]
    found = False

    while len(heap) > 0:
        item = heapq.heappop(heap)
        current_node = item[1]

        if same_node(current_node, goal):
            print('Found a path.')
            found = True
            break

        if not same_node(current_node, start):
            current_cost = branch[current_node][0]
        else:
            current_cost = 0.0

        for action in possible_actions:
            if action.is_valid(grid, current_node):
                # get the tuple representation
                dx, dy = action.delta
                nx, ny = current_node[0] + dx, current_node[1] + dy
                next_node = (nx, ny)
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node, action)
                    heapq.heappush(heap, (queue_cost, next_node))

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


def heuristic(position: GridPosition, goal_position: GridPosition) -> float:
    dx = position[0] - goal_position[0]
    dy = position[1] - goal_position[1]
    delta_sq = dx*dx + dy*dy
    return np.sqrt(delta_sq)

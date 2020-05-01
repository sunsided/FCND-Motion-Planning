import numpy as np
from planning_utils import GridPosition


def line_test(grid: np.ndarray,  p0: GridPosition, p1: GridPosition) -> bool:
    """
    Tests whether the path between two points is safe to maneuver, given the grid, by performing a ray cast.
    :param grid: The grid to test with.
    :param p0: The start point.
    :param p1: The goal point.
    :return: True if the line is entirely in free space; False otherwise.
    """

    # This is based on the bresenham Python package implementation by
    # Petr Viktorin at https://github.com/encukou/bresenham

    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]

    xsign = 1 if dx > 0 else -1
    ysign = 1 if dy > 0 else -1

    dx = abs(dx)
    dy = abs(dy)

    if dx > dy:
        xx, xy, yx, yy = xsign, 0, 0, ysign
    else:
        dx, dy = dy, dx
        xx, xy, yx, yy = 0, ysign, xsign, 0

    D = 2*dy - dx
    y = 0

    for x in range(dx + 1):
        point_x = p0[0] + x*xx + y*yx
        point_y = p0[1] + x*xy + y*yy
        if grid[point_x, point_y] != 0:
            return False

        if D >= 0:
            y += 1
            D -= 2*dx
        D += 2*dy

    return True

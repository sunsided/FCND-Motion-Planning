import unittest
import pytest
import numpy as np
from motion_planning import MotionPlanning


@pytest.mark.parametrize("test_input,expected",
                         [(0, 0), (-1, -1),
                          (90, 90), (-90, -90),
                          (180, -180), (-180, -180), (-181, 179), (181, -179),
                          (360, 0), (359, -1), (-359, 1),
                          (720, 0), (-720, 0)])
def test_angle_wrapping(test_input, expected):
    assert np.isclose(MotionPlanning.wrap_angle(np.deg2rad(test_input)), np.deg2rad(expected))


def test_opposite_waypoints_have_zero_heading():
    p0 = [0, 10, 42, 0]  # from the right
    p1 = [0, 0, 42, 0]   # goes left
    p2 = [0, 10, 42, 0]  # goes right again
    wps = [p0, p1, p2]
    MotionPlanning.interpolate_headings(wps, fix_first=True, fix_last=True)
    assert(p0[3] == 0 and p1[3] == 0 and p2[3] == 0)


def test_sequential_waypoints_have_identical_heading():
    wps = [[0, east, 42, 0] for east in range(100)]
    MotionPlanning.interpolate_headings(wps, fix_first=True, fix_last=True)
    assert(np.all([np.isclose(wp[3], wps[0][3]) for wp in wps]))


if __name__ == '__main__':
    unittest.main()

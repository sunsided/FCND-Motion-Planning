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


if __name__ == '__main__':
    unittest.main()

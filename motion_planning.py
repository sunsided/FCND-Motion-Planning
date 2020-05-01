# -*- coding: utf-8 -*-
"""
Solution to the 3D Motion Planning project.
"""

import argparse
import time
import msgpack
import re
from enum import Enum, auto
from typing import Optional, Union, Tuple, List, NamedTuple

import numpy as np

from planning_utils import a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


LatLon = NamedTuple('LatLon', [('latitude', float), ('longitude', float)])


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.colliders_file = 'colliders.csv'
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.flight_state = None  # type: Optional[States]
        self.verbose = False

        # convencience vector to flip up with down
        self.flip_ud = np.array([1, 1, -1], dtype=float)

        # map data
        self.grid = None  # type: Optional[np.ndarray]
        self.height_map = None  # type: Optional[np.ndarray]
        self.grid_offset = (0., 0.)
        self.target_altitude = 5  # type: int
        self.safety_distance = 5  # type: int

        # initial state
        self.set_state(States.MANUAL)

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def set_verbose(self, verbose: bool):
        self.verbose = verbose

    def close_to_target_ne(self, deadband: float) -> bool:
        return np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < deadband

    def close_to_target_ned(self, deadband: float) -> bool:
        return np.linalg.norm(self.target_position[0:3] - self.local_position[0:3] * self.flip_ud) < deadband

    def in_state(self, state: States) -> bool:
        return state == self.flight_state

    def set_state(self, state: States):
        if self.verbose:
            print(f'Going from {self.flight_state} to {state}')
        self.flight_state = state

    def local_position_callback(self):
        if self.in_state(States.TAKEOFF):
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.in_state(States.WAYPOINT):
            if self.close_to_target_ned(1.0):
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.in_state(States.LANDING):
            # This condition is problematic: Comparing to global home base height
            # implies that we need to land on exactly the same height. The
            # follow-up check makes it even worse. If we were to land on, say,
            # a roof, this would never allow us to disarm the drone.
            # TODO: Allow disarming when in rest.
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.in_state(States.MANUAL):
                self.arming_transition()
            elif self.in_state(States.ARMING):
                if self.armed:
                    self.plan_path()
            elif self.in_state(States.PLANNING):
                self.takeoff_transition()
            elif self.in_state(States.DISARMING):
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.set_state(States.ARMING)
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.set_state(States.TAKEOFF)
        print(f"takeoff transition to {self.target_altitude} m")
        self.takeoff(self.target_altitude)

    def waypoint_transition(self):
        self.set_state(States.WAYPOINT)
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.set_state(States.LANDING)
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.set_state(States.DISARMING)
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.set_state(States.MANUAL)
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)

        # A bug waiting to happen here, but this is how the original code
        # implemented sending data via MAVLink.
        # noinspection PyProtectedMember
        self.connection._master.write(data)

    def plan_path(self):
        self.set_state(States.PLANNING)
        self.target_position[2] = self.target_altitude

        # Rubric points:
        # - Read lat0, lon0 from colliders into floating point values
        # - Set home position to (lon0, lat0, 0)
        home_latlon = self.read_lat0lon0(self.colliders_file)
        self.set_home_position(home_latlon.longitude, home_latlon.latitude, 0)

        # Rubric points:
        # - Retrieve current global position
        # - Convert to current local position using global_to_local()
        geodetic_coords = [self._longitude, self._latitude, self._altitude]
        local_position = global_to_local(global_position=geodetic_coords, global_home=self.global_home)
        local_position_difference = np.linalg.norm(np.subtract(self.local_position, local_position))

        # The values do differ; let's just make sure here that nothing too weird is going on.
        assert (np.isclose(local_position_difference, 0, rtol=1, atol=1)), \
            f"Calculated local position {local_position} differs " \
            f"from provided one {self.local_position} by {local_position_difference} m."

        print('global home {0}, position {1}, local position {2} (differs by {3:.4} m)'.format(
            self.global_home, self.global_position, self.local_position, local_position_difference))

        # Read in obstacle map
        data = np.loadtxt(self.colliders_file, delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, heightmap, north_offset, east_offset = create_grid(data, self.target_altitude, self.safety_distance)
        self.set_map(grid, heightmap, north_offset, east_offset)

        print("Searching for a path ...")

        # Define starting point on the grid (this is just grid center)
        # Rubric point: convert start position to current position rather than map center
        # NOTE that I used the self.local_position here instead of the local_position
        #      we just calculated manually. The values do seem to diverge, so I'll simply
        #      rely on what the Drone base class assumes to be correct.
        grid_start = (int(self.local_position[0] - north_offset), int(self.local_position[1] - east_offset))

        # Set goal as some arbitrary position on the grid
        # TODO: adapt to set goal as latitude / longitude position and convert
        grid_goal = (-north_offset + 10, -east_offset + 10)

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(self.grid, heuristic, grid_start, grid_goal)
        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!

        # Convert path to waypoints
        waypoints = [self.to_waypoint(p, north_offset, east_offset, self.target_altitude) for p in path]
        self.interpolate_headings(waypoints, fix_first=True)
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def read_lat0lon0(self, colliders_file: str) -> LatLon:
        """
        Parses the home latitude and longitude from the provided colliders file.
        :param colliders_file: The colliders file to load.
        :return: The home latitude and longitude.
        """
        with open(colliders_file, 'r') as f:
            line = f.readline()
            return self.parse_lat0lon0(line)

    @staticmethod
    def parse_lat0lon0(line: str) -> LatLon:
        """
        Parses the home latitude and longitude from the provided string.
        :param line: The line to parse.
        :return: The home latitude and longitude.
        """
        match = re.match(r'lat0 (?P<lat>[+-]?\d+(\.\d*)?), lon0 (?P<lon>[+-]?\d+(\.\d*)?)', line, re.IGNORECASE)
        assert match, f'Invalid input: {line}'
        lat = float(match.group('lat'))
        lon = float(match.group('lon'))
        return LatLon(latitude=lat, longitude=lon)

    @staticmethod
    def to_waypoint(point: Union[List[float], Tuple[float, float, float, float], np.ndarray],
                    north_offset: float, east_offset: float, altitude: float) -> List[float]:
        return [point[0] + north_offset, point[1] + east_offset, altitude, 0]

    @staticmethod
    def wrap_angle(theta: float) -> float:
        """
        Converts an angle to -pi .. pi range (so 181 degree becomes -179, and -181 degree becomes 179).
        :param theta: The angle in radians.
        :return: The angle in radians wrapped to the range of -pi .. pi.
        """
        return (theta + np.pi) % (2*np.pi) - np.pi

    @staticmethod
    def interpolate_headings(waypoints: List[Union[List[float], Tuple[float, float, float, float], np.ndarray]],
                             fix_first: bool = False, fix_last: bool = True) -> None:
        """
        Interpolates headings between two adjacent waypoints.
        :param waypoints: The waypoint list to update in place.
        :param fix_first: Whether to correct the first waypoint in the list.
        :param fix_last: Whether to correct the last waypoint in the list.
        """
        # This uses a naive scheme to interpolate headings between two adjacent
        # waypoints. We could trivially set each waypoint's heading to be
        # the angle from the previous or to the next one. However, this would
        # result in constantly alternating headings on a simple, unpruned,
        # nondiagonal-movement-only planning grid as it was used in the starter
        # code. By interpolating the headings between the previous and next
        # waypoint, motion should be a lot smoother even under bad conditions.
        # When following a "staircase" like path, all headings will be diagonal
        # along the mean trajectory.
        for w in range(1, len(waypoints)-1):
            p0 = waypoints[w-1]
            p1 = waypoints[w]
            p2 = waypoints[w+1]

            theta_0_1 = np.arctan2((p1[1] - p0[1]), (p1[0] - p0[0]))
            theta_1_0 = np.arctan2((p2[1] - p1[1]), (p2[0] - p1[0]))

            theta = MotionPlanning.wrap_angle((theta_0_1 + theta_1_0) * 0.5)
            p1[3] = theta

        # Set very last waypoint to the previous waypoint's heading. If no new waypoints
        # are coming, this will smoothen the movement. If new waypoints are added, it will
        # eventually be updated with an interpolated value.
        # Since we remove waypoints from the beginning of the list, we leave the first
        # waypoint untouched.
        if len(waypoints) > 1:
            waypoints[0][3] = waypoints[1][3] if fix_first else waypoints[0][3]
            waypoints[-1][3] = waypoints[-2][3] if fix_last else waypoints[-1][3]

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()

    @property
    def map_loaded(self):
        return self.grid is not None

    def set_map(self, grid: np.ndarray, heightmap: np.ndarray, north_offset: float, east_offset: float) -> None:
        """
        Sets the map
        :param grid: The discretized grid.
        :param heightmap: The 2.5-d map of the scenery.
        :param north_offset: The north offset of the grid into the map.
        :param east_offset: The east offset of the grid into the map.
        """
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        self.grid = grid
        self.height_map = heightmap
        self.grid_offset = (north_offset, east_offset)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('-v', '--verbose', dest='verbose', default=False, action='store_true', help='Enables verbose logging')
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    drone.set_verbose(args.verbose)

    time.sleep(1)

    drone.start()

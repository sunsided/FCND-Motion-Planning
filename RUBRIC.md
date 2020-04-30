# Project Specification: 3D Motion Planning

## Writeup

- **Criteria:**
  Provide a Writeup / README that includes all the rubric points and how you addressed each one.
  You can submit your writeup as markdown or PDF.

  **Meets specifications:**
  The writeup / README should include a statement and supporting figures / images that explain
  how each rubric item was addressed, and specifically where in the code each step was handled.

## Explain the Starter Code

- **Criteria:**
  Test that `motion_planning.py` is a modified version of `backyard_flyer.py` for simple
  path planning. Verify that both scripts work. Then, compare them side by side and describe in
  words how each of the modifications implemented in `motion_planning.py` is functioning.

  **Meets specifications:**
  The goal here is to understand the starter code. We've provided you with a functional yet super
  basic path planning implementation and in this step, your task is to explain how it works!
  Have a look at the code, particularly in the `plan_path()` method and functions provided in
  `planning_utils.py` and describe what's going on there. This need not be a lengthy essay,
  just a concise description of the functionality of the starter code.

## Implementing Your Path Planning Algorithm

- **Criteria:**
  In the starter code, we assume that the home position is where the drone first initializes,
  but in reality you need to be able to start planning from anywhere. Modify your code to read
  the global home location from the first line of the `colliders.csv` file and set that position
  as global home (`self.set_home_position()`).

  **Meets specifications:**
  Here you should read the first line of the CSV file, extract `lat0` and `lon0` as floating point
  values and use the `self.set_home_position()` method to set global home.

- **Criteria:**
  In the starter code, we assume the drone takes off from map center, but you'll need to be able
  to takeoff from anywhere. Retrieve your current position in geodetic coordinates from
  `self._latitude`, `self._longitude` and `self._altitude`. Then use the utility function
  `global_to_local()` to convert to local position (using `self.global_home` as well, 
  which you just set).

  **Meets specifications:**
  Here as long as you successfully determine your local position relative to global home
  you'll be all set.

- **Criteria:**
  In the starter code, the `start` point for planning is hardcoded as map center.
  Change this to be your current local position.
  
  **Meets specifications:**
  This is another step in adding flexibility to the start location.
  As long as it works you're good to go!

- **Criteria:**
  In the starter code, the goal position is hardcoded as some location 10 m north and 10 m east
  of map center. Modify this to be set as some arbitrary position on the grid given any geodetic
  coordinates (latitude, longitude).

  **Meets specifications:**
  Minimal requirement here is to modify the code in `planning_utils()` to update the A*
  implementation to include diagonal motions on the grid that have a cost of `sqrt(2)`,
  but more creative solutions are welcome. In your writeup, explain the code you used to
  accomplish this step.

- **Criteria:**
  Cull waypoints from the path you determine using search.

  **Meets specifications:**
  For this step you can use a collinearity test or ray tracing method like Bresenham.
  The idea is simply to prune your path of unnecessary waypoints. In your writeup, explain
  the code you used to accomplish this step.
  
## Executing the flight

- **Criteria:**
  This is simply a check on whether it all worked. Send the waypoints and the autopilot
  should fly you from start to goal!

  **Meets specifications:**
  At the moment there is some mismatch between the colliders map and actual buildings in the
  scene. To ensure success build in a 5+ m safety margin around obstacles. Try some different
  goal locations. Also try starting from a different point in the city. Your reviewer will also
  try some random locations so be sure to test your solution! There is no firm constraint or
  requirement on how accurately you land exactly on the goal location. Just so long as your
  planner functions as expected.

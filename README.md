# FCND - 3D Motion Planning
![Quad Image](./misc/enroute.png)

This project is a continuation of the [Backyard Flyer](https://github.com/sunsided/FCND-Backyard-Flyer)
project where we executed a simple square shaped flight path. 
It integrates techniques to plan a path through an urban environment. Check out the [project rubric](RUBRIC.md)
([original link](https://review.udacity.com/#!/rubrics/1534/view)) for more detail on how this implementation
is differing from the starter code forked off the [original repo](https://github.com/udacity/FCND-Backyard-Flyer).

## Environment

This project is running in a simulated environment. You can download the Linux version of the simulator from from the
releases of the [sunsided/fcnd-notebooks](https://github.com/sunsided/fcnd-notebooks) GitHub repo.

The code was implemented against the Conda environment described in `environment.yml`. To recreate the environment,
run

```bash
conda env create -f environment. yml
```

After that, activate it using

```bash
conda activate udacity-fcnd
```

## Running the code

Start the simulator and select the _Motion Planning_ scene. Once the scene is initialized,
execute the Python code by running

```bash
python motion_planning.py
```

Note that the file `backyard_flyer.py` is the solution to the [Backyard Flyer](https://github.com/sunsided/FCND-Backyard-Flyer)
project and is meant to be run against the _Backyard Flyer_ simulator scene.
It is kept here for reference, as the project rubric requires it.

## Project Description

### Starter Code

As mentioned above, `motion_planning.py` is based on the `backyard_flyer.py` solution
which, itself, involves taking off, flying a square, then landing safely.

As before, waypoints are calculated exactly once in the beginning of the mission.
In both solutions, they are issued individually by calling the `Drone` class' `cmd_position(north, east, altitude, heading)`;
here however, we're now sending off an entire list of waypoints in addition for visualization 
by calling `send_waypoints()`. This, in turn, makes use of a somewhat undocumented
functionality of the underlying [MAVLink](https://github.com/ArduPilot/pymavlink) connection.

What used to be `calculate_box()` in the Backyard Flyer was now replaced with `plan_path()`.
For this, an extra state `PLANNING` was introduced in the `States` enum, to be executed between
`ARMING` and `TAKEOFF`. 

Specifically, the `backyard_flyer.py` solution code has these state transitions:

![States of the Backyard Flyer solution](misc/backyard-flyer-states.png)

For comparison, state transitions of the `motion_planning.py` starter code look like this:

![States of the Motion Planning starter code](misc/motion-planner-states.png)

The following output of `motion_planning.py` when run off-the-shelf (as of commit `bb51472b`) gives
a nice overview of the general process. Note that some lines were removed for brevity.

```
Logs/NavLog.txt
starting connection
arming transition
Searching for a path ...
global home [-122.39745   37.79248    0.     ], position [-122.3974494   37.7924801    0.152    ], local position [ 0.02221821  0.05091386 -0.15224203]
North offset = -316, east offset = -445
Local Start and Goal:  (316, 445) (326, 455)
Found a path.
Sending waypoints to simulator ...
takeoff transition
waypoint transition
target position [0, 0, 5, 0]
waypoint transition
target position [0, 1, 5, 0]
waypoint transition
target position [1, 1, 5, 0]
waypoint transition
... etc. ...
target position [9, 10, 5, 0]
waypoint transition
target position [10, 10, 5, 0]
landing transition
disarm transition
manual transition
Closing connection ...
```

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

Note that the file `backyard_flyer_solution.py` is the solution to the [Backyard Flyer](https://github.com/sunsided/FCND-Backyard-Flyer)
project and is meant to be run against the _Backyard Flyer_ simulator scene.
It is kept here for reference, as the project rubric requires it.

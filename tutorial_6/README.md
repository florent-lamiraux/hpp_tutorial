# How to have a rviz2 visualization

## Prerequisite

Having completed [tutorial 5](../tutorial_5/README.md).

## Overview

This tutorial shows how to set up an RViz2 visualization for HPP using the `RVizVisualizer`
from `pyhpp_rviz`. Unlike the web-based viewer used in tutorials 2–5, this visualizer
communicates over ROS 2 topics and displays the robot, paths, and waypoints directly inside
RViz2.

## Setting up the simulation

The base tutorial docker image does not include ROS 2 control packages. Build the
extended image from the tutorial 6 directory **on the host machine** (not inside
the container):

```
cd tutorial_6
docker build --build-arg DOCKER_USER=`id -u` --build-arg DOCKER_GROUP=`id -g` \
    -t hpp-ros2:tuto .
```

Then start the container from the root shared directory:

```
cd ../../..
./src/hpp_tutorial/tutorial_6/run_docker.sh
```

## Compiling the HPP RViz2 plugins

On your first `make all` from tutorial 1, the RViz2 plugin sources were fetched but not
compiled. Rebuild `hpp-gepetto-viewer` with the RViz2 flag enabled:

```
cd src
make hpp-gepetto-viewer_extra_flags="\
        -DINSTALL_DOCUMENTATION=OFF     \
        -DUSE_HPP_PYTHON=ON             \
        -DPYTHON_STANDARD_LAYOUT=ON     \
        -DBUILD_HPP_RVIZ_PKGS=ON"       \
        hpp-gepetto-viewer.very-clean   \
    && make hpp-gepetto-viewer.install
```

export the package path

```
export ROS_PACKAGE_PATH=/home/user/devel/src/:$ROS_PACKAGE_PATH
```

## Initializing the problem

In the docker container, cd into `tutorial_6` directory and run:

```
python -i init.py
```

The script is identical to tutorial 5 up to the computation of paths `p1`, `p2`, `fullPath`.
The only difference is that it uses `RVizVisualizer` instead of `Viewer`.

## Configuring RViz2

Open a second terminal in the container:

```
docker exec -it hpp bash
rviz2
```

In RViz2, set the **Fixed Frame** to `world` (in the *Global Options* panel).

Set the Frame Rate to 300

For each model loaded with `urdf.loadModel`, add a **RobotModel** display and set its
`Description Topic` to the topic published for that model (e.g. `/staubli/robot_description`,
`/plate/robot_description`, `/obstacle/robot_description`). The prefix matches the name
passed to `urdf.loadModel`.

Add a **TF** display to visualize all frame transforms published by the viewer.

Disable TF arrows and put Frame Time Out to 1e+07

There is a rviz config file on hpp_tutorial/launch/tuto6.rviz
```bash
rviz2 -d hpp_tutorial/launch/tuto6.rviz
```

Call `v(q_init)` in the Python terminal to place all objects in their initial configuration.
The scene should now appear in RViz2.

## Visualizing a path

Add the **Trajectory** plugin (from the HPP RViz2 plugins) to RViz2. A control panel
will appear at the bottom of the screen.

Load a path from the Python terminal:

```python
v.loadPath(p1)
```

The panel lets you slide through the path parameter to inspect any intermediate configuration.

To display the spatial trace of a frame along the path, use:

```python
v.displayPath(p1, target_frame="staubli/tooltip")
```

You can also trigger this from RViz2 by entering the frame name directly in the
DisplayTrajectory panel.

## Adding waypoints

Add the **Waypoint** tool from the RViz2 toolbar (installed with the HPP RViz2 plugins).
Add the **DisplayWaypoint** display to visualize the waypoints in the scene.

Waypoints can be placed in three ways:

- **Interactively**: select the Waypoint tool in the toolbar, then click in the 3D view.
  Drag the waypoint with the interact tool for moving it or
  Right-click a waypoint marker and choose *Edit position* to adjust it numerically with the intrect tool too

- **From a named frame** (Python):
  ```python
  v.addWaypointFromFrame("staubli/tooltip")
  ```
  This publishes the current pose of the given frame as a waypoint.

- **From explicit coordinates** (Python):
  ```python
  v.addWaypoint(xyz=[0.8, 0.0, 1.0], quat_xyzw=[0.0, 0.0, 0.0, 1.0])
  ```

## Summary of viewer methods

| Method | Description |
|---|---|
| `v(q)` | Display configuration `q` |
| `v.loadPath(p)` | Register path `p` for trajectory control |
| `v.displayPath(p, target_frame=...)` | Publish the spatial trace of a frame along `p` |
| `v.addWaypointFromFrame(frame)` | Publish current pose of `frame` as a waypoint |
| `v.addWaypoint(xyz, quat_xyzw)` | Publish an explicit pose as a waypoint |
| `v.setProblem(problem)` | Register problem for graph viewer |
| `v.setGraph(graph)` | Register constraint graph for graph viewer |
| `v.launch_graph_viewer()` | Open the constraint graph viewer (React app) |

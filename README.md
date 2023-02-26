# Dijkstra path planner application

Simple path planning application using Dijkstra's path planning algorithm.


<p align="center">
  <img src="https://github.com/HrushikeshBudhale/path_planner_app/blob/main/doc/path_planner.gif?raw=0" alt="OUTPUT GIF" width="600"/>
</p>

## Dependencies
- python 3.6 or above
- numpy
- matplotlib

## How to run?

Open the terminal at the location of ```dijkstra_planner.py``` file, and enter

    python3 dijkstra_planner.py


Running this command will open a window with available region and obstacles
1. Click inside the available region to select start point
2. Again click at different location inside available region to select goal point
3. (optional) Adjust slider to change the speed of exploration animation

## Misc.

- Implementation takes ~4.5 sec to find path between farthest points among 100K nodes.

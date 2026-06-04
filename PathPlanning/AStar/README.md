# A* Grid Planner Notes

This folder contains the 2D grid-based A* planner.

## Basic usage

```python
from PathPlanning.AStar.a_star import AStarPlanner, create_demo_obstacle_map

ox, oy = create_demo_obstacle_map()
planner = AStarPlanner(ox, oy, resolution=2.0, rr=1.0)
rx, ry = planner.planning(10.0, 10.0, 50.0, 50.0)
```

`rx` and `ry` are returned from goal to start, matching the existing planner
behavior.

## Weighted A*

The default `heuristic_weight=1.0` preserves the original A* behavior.
Increasing the weight makes the heuristic greedier and can reduce expanded
nodes. Measure the path length trade-off before using a higher value.

```python
planner = AStarPlanner(
    ox, oy, resolution=2.0, rr=1.0,
    heuristic_weight=1.2,
)
```

## Tie breaking

Set `tie_breaker="larger_g"` to prefer nodes farther from the start when two
nodes have the same priority. Leave it as `None` for the default behavior.

```python
planner = AStarPlanner(
    ox, oy, resolution=2.0, rr=1.0,
    heuristic_weight=1.2,
    tie_breaker="larger_g",
)
```

## Visualization options

The following options are disabled by default and only apply when
`show_animation` is enabled in `a_star.py`:

- `show_open_set`: display frontier nodes.
- `show_closed_set`: display explored nodes.
- `show_cost_heatmap`: overlay expanded-node costs after planning.
- `show_path_progress`: display the current parent chain during search.

```python
planner = AStarPlanner(
    ox, oy, resolution=2.0, rr=1.0,
    show_open_set=True,
    show_closed_set=True,
    show_cost_heatmap=True,
    show_path_progress=True,
)
```

## Benchmark

From the bootcamp root, compare the default planner with weighted variants:

```powershell
cd "./PythonRobotics"
python "../tools/benchmark_a_star.py" --repeat 5
```

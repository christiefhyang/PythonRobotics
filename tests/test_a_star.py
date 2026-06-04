import contextlib
import io
import math

import pytest

import conftest
from PathPlanning.AStar import a_star as m


def test_1():
    m.show_animation = False
    m.main()


def create_test_map():
    return m.create_demo_obstacle_map()


def plan_path(**planner_options):
    ox, oy = create_test_map()
    planner = m.AStarPlanner(ox, oy, 2.0, 1.0, **planner_options)
    with contextlib.redirect_stdout(io.StringIO()):
        rx, ry = planner.planning(10.0, 10.0, 50.0, 50.0)

    return planner, rx, ry


def calc_path_length(rx, ry):
    return sum(math.hypot(rx[i] - rx[i - 1], ry[i] - ry[i - 1])
               for i in range(1, len(rx)))


def test_weight_one_keeps_default_path():
    m.show_animation = False

    _, default_rx, default_ry = plan_path()
    _, weighted_rx, weighted_ry = plan_path(heuristic_weight=1.0)

    assert weighted_rx == default_rx
    assert weighted_ry == default_ry
    assert calc_path_length(weighted_rx, weighted_ry) == pytest.approx(
        calc_path_length(default_rx, default_ry))


def test_weighted_a_star_returns_valid_path():
    m.show_animation = False

    planner, rx, ry = plan_path(heuristic_weight=1.5,
                                tie_breaker="larger_g")

    assert rx[0] == pytest.approx(50.0)
    assert ry[0] == pytest.approx(50.0)
    assert rx[-1] == pytest.approx(10.0)
    assert ry[-1] == pytest.approx(10.0)
    assert calc_path_length(rx, ry) > 0.0
    assert planner.last_expanded_node_count > 0


def test_invalid_a_star_options():
    ox, oy = create_test_map()

    with pytest.raises(ValueError):
        m.AStarPlanner(ox, oy, 2.0, 1.0, heuristic_weight=0.0)

    with pytest.raises(ValueError):
        m.AStarPlanner(ox, oy, 2.0, 1.0, tie_breaker="unknown")


def test_visualization_options_keep_planning_usable():
    m.show_animation = False

    planner, rx, ry = plan_path(show_open_set=True,
                                show_closed_set=True,
                                show_path_progress=True,
                                show_cost_heatmap=True)

    assert len(rx) == len(ry)
    assert len(rx) > 0
    assert planner.show_open_set
    assert planner.show_closed_set
    assert planner.show_path_progress
    assert planner.show_cost_heatmap


def test_create_demo_obstacle_map():
    ox, oy = m.create_demo_obstacle_map()

    assert len(ox) == len(oy)
    assert len(ox) > 0


if __name__ == '__main__':
    conftest.run_this_test(__file__)

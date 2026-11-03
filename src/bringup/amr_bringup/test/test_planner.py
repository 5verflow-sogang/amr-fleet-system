"""전역 planner 고르기 (amr_bringup.planner). ROS 없이 돈다."""

import os

import pytest
import yaml

from amr_bringup.planner import planner_overrides

PARAMS = os.path.join(os.path.dirname(__file__), '..', 'config', 'nav2_params.yaml')


@pytest.fixture(scope='module')
def nav2_params():
    with open(PARAMS) as f:
        return yaml.safe_load(f)


def test_navfn_is_default_and_adds_nothing(nav2_params):
    planner = nav2_params['planner_server']['ros__parameters']
    assert planner['planner_plugins'] == ['GridBased']
    assert planner['GridBased']['plugin'] == 'nav2_navfn_planner/NavfnPlanner'
    assert planner_overrides(nav2_params, 'navfn') == {}


def test_astar_replaces_grid_based_with_the_astar_block(nav2_params):
    overrides = planner_overrides(nav2_params, 'astar')
    assert overrides['GridBased.plugin'] == 'amr_nav2_plugins::AStarGlobalPlanner'
    block = nav2_params['planner_server']['ros__parameters']['AStar']
    assert {k[len('GridBased.'):] for k in overrides} == set(block)
    assert overrides['GridBased.lethal_cost_threshold'] == 253
    assert overrides['GridBased.smooth_path'] is True


def test_unknown_planner_is_rejected(nav2_params):
    with pytest.raises(ValueError):
        planner_overrides(nav2_params, 'smac')

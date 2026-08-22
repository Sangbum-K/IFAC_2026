"""Unit tests for the obstacle bridge's source-specific filtering policy."""

import sys
from pathlib import Path


PLANNER_SCRIPTS = Path(__file__).resolve().parents[1] / "scripts" / "planner"
sys.path.insert(0, str(PLANNER_SCRIPTS))

from obstacle_bridge_policy import (  # noqa: E402
    StaticHoldBook,
    static_hold_ids_for_dynamic_source,
    visible_static_wall_distance_minimum,
)


def test_wall_slack_is_limited_to_measured_small_static_obstacles():
    base = 0.30
    relaxed = visible_static_wall_distance_minimum(
        base, 0.05, True, False, 0.25, 0.25)
    assert relaxed == 0.25

    for is_visible, is_dynamic, half_extent in (
            (False, False, 0.25),
            (True, True, 0.25),
            (True, False, 0.251)):
        assert visible_static_wall_distance_minimum(
            base, 0.05, is_visible, is_dynamic, half_extent, 0.25) == base


def test_static_hold_is_bounded_and_live_observations_win():
    holds = StaticHoldBook(0.75)
    payload = object()
    holds.remember(42, 10.0, payload)

    assert holds.missing(10.30, set()) == [(42, payload)]
    assert holds.missing(10.30, {42}) == []
    assert holds.missing(10.76, set()) == []


def test_static_hold_forgets_dynamic_reclassification():
    holds = StaticHoldBook(0.75)
    holds.remember(42, 10.0, object())
    holds.forget(42)

    assert holds.missing(10.10, set()) == []


def test_static_hold_forgets_reassociated_dynamic_source():
    holds = StaticHoldBook(0.75)
    holds.remember(42, 10.0, object())

    # A new detector id had been absorbed into static track 42. A dynamic
    # update for source id 100 must remove both aliases before it is emitted.
    assert static_hold_ids_for_dynamic_source(100, 42) == (100, 42)
    holds.forget(*static_hold_ids_for_dynamic_source(100, 42))

    assert holds.missing(10.10, {100}) == []

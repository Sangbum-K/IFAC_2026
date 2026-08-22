"""Unit tests for the obstacle bridge's source-specific filtering policy."""

import sys
from pathlib import Path


PLANNER_SCRIPTS = Path(__file__).resolve().parents[1] / "scripts" / "planner"
sys.path.insert(0, str(PLANNER_SCRIPTS))

from obstacle_bridge_policy import (  # noqa: E402
    bridge_output_is_static,
    DynamicFrenetWitness,
    DynamicShadowQuarantine,
    StaticHoldBook,
    static_candidate_is_in_dynamic_shadow,
    static_source_is_new,
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


def test_h2h_dynamic_shadow_catches_both_recorded_wall_ghost_offsets():
    """#108/#110 were new static sources near raw dynamic #105."""
    opponent = DynamicFrenetWitness(10.0, 0.0)
    limits = dict(
        dynamic_witnesses=[opponent], track_length=40.0,
        ahead_limit=2.50, behind_limit=0.25, lateral_limit=1.00)

    # run_0822_hth birth offsets relative to #105: #108 (+1.37, +0.97),
    # #110 (+2.18, -0.80).  Neither is close enough for track reassociation.
    assert static_candidate_is_in_dynamic_shadow(11.37, 0.97, **limits)
    assert static_candidate_is_in_dynamic_shadow(12.18, -0.80, **limits)

    # Do not turn this into a broad static-obstacle deletion rule.
    assert not static_candidate_is_in_dynamic_shadow(12.51, 0.20, **limits)
    assert not static_candidate_is_in_dynamic_shadow(9.74, 0.20, **limits)
    assert not static_candidate_is_in_dynamic_shadow(11.00, 1.01, **limits)


def test_h2h_dynamic_shadow_is_order_independent_and_wraps_s():
    near = DynamicFrenetWitness(39.90, -0.20)
    far = DynamicFrenetWitness(20.00, 0.00)
    kwargs = dict(track_length=40.0, ahead_limit=2.50,
                  behind_limit=0.25, lateral_limit=1.00)

    assert static_candidate_is_in_dynamic_shadow(
        0.10, -0.20, [near, far], **kwargs)
    assert static_candidate_is_in_dynamic_shadow(
        0.10, -0.20, [far, near], **kwargs)


def test_h2h_shadow_quarantine_is_bounded_and_new_source_only():
    quarantine = DynamicShadowQuarantine(2.50)

    # A current dynamic shadow starts a bounded lease for a brand-new source.
    assert quarantine.should_quarantine(110, 100.0, True, True)
    assert quarantine.is_active(110, 102.49)
    # The source cannot reappear in the brief gap after the opponent leaves.
    assert quarantine.should_quarantine(110, 102.49, True, False)
    assert not quarantine.should_quarantine(110, 102.51, True, False)

    # A confirmed static track is never hidden merely because an opponent is
    # nearby; calling this also clears any stale quarantine lease for its ID.
    assert not quarantine.should_quarantine(42, 103.0, False, True)
    assert not quarantine.is_active(42, 103.0)


def test_h2h_shadow_preserves_reassociated_confirmed_static_track():
    # Detector ID 100 is new, but it was already mapped to mature bridge track
    # 42.  It must not be treated as a new shadow candidate.
    known_tracks = {42: object()}
    assert not static_source_is_new(100, known_tracks, 42)
    assert static_source_is_new(100, known_tracks, None)
    assert static_source_is_new(100, known_tracks, 77)


def test_h2h_profile_can_preserve_raw_dynamic_through_confidence_reset():
    # Legacy mode remains available, but H2H must not turn raw dynamic #105
    # into a static obstacle when classifier confidence briefly resets.
    assert bridge_output_is_static(True, True, True)
    assert not bridge_output_is_static(True, True, False)
    assert not bridge_output_is_static(True, False, True)
    assert bridge_output_is_static(False, False, False)


def test_h2h_quarantined_source_cannot_return_through_static_hold():
    quarantine = DynamicShadowQuarantine(2.50)
    holds = StaticHoldBook(0.75)
    holds.remember(110, 100.0, object())
    assert quarantine.should_quarantine(110, 100.0, True, True)

    if quarantine.is_active(110, 100.2):
        holds.forget(110)
    assert holds.missing(100.2, set()) == []

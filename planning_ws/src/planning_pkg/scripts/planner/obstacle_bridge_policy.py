"""Small, ROS-free policies used by the perception obstacle bridge."""

from dataclasses import dataclass


def visible_static_wall_distance_minimum(
        base_minimum, visible_static_slack, is_visible, is_dynamic,
        raw_half_extent, static_extent_limit):
    """Return the EDT cutoff for one raw perception obstacle.

    The relaxed cutoff is deliberately limited to a measured, static, small
    obstacle. Dynamic, coasting, and large clusters keep the full wall guard.
    """
    if (is_visible and not is_dynamic
            and raw_half_extent <= static_extent_limit):
        return max(0.0, base_minimum - visible_static_slack)
    return base_minimum


def static_hold_ids_for_dynamic_source(source_id, reassociated_track_id=None):
    """Return every static-hold ID invalidated by a dynamic source update."""
    hold_ids = [int(source_id)]
    if reassociated_track_id is not None:
        hold_ids.append(int(reassociated_track_id))
    return tuple(hold_ids)


@dataclass
class _StaticHold:
    observed_at: float
    payload: object


class StaticHoldBook:
    """Bounded lease for the last measured static obstacle payload."""

    def __init__(self, hold_sec):
        self.hold_sec = float(hold_sec)
        self._holds = {}

    def remember(self, track_id, observed_at, payload):
        if self.hold_sec > 0.0:
            self._holds[int(track_id)] = _StaticHold(
                float(observed_at), payload)

    def forget(self, *track_ids):
        for track_id in track_ids:
            self._holds.pop(int(track_id), None)

    def missing(self, now_sec, live_ids):
        """Return unexpired holds not superseded by this frame's live IDs."""
        now_sec = float(now_sec)
        live_ids = {int(track_id) for track_id in live_ids}
        missing = []
        for track_id, hold in list(self._holds.items()):
            age = now_sec - hold.observed_at
            if age < 0.0 or age > self.hold_sec:
                del self._holds[track_id]
            elif track_id not in live_ids:
                missing.append((track_id, hold.payload))
        return missing

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


@dataclass(frozen=True)
class DynamicFrenetWitness:
    """One raw, measured dynamic obstacle from the current source frame.

    This deliberately models the *input* classification, rather than the
    bridge output's ``is_static`` field.  Legacy profiles may emit a
    low-confidence dynamic input as static for downstream planning, but it is
    still valid evidence that a nearby brand-new static candidate may be the
    same wall/cluster artefact in a head-to-head pass.
    """

    s: float
    d: float


def static_candidate_is_in_dynamic_shadow(
        static_s, static_d, dynamic_witnesses, track_length,
        ahead_limit, behind_limit, lateral_limit):
    """Whether a static candidate lies in a current-frame dynamic shadow.

    ``dynamic_witnesses`` must contain only visible, raw-dynamic observations
    from the same source frame.  Keeping this frame-local prevents an old or
    coasting opponent track from hiding a real static obstacle after it has
    passed.  The bridge applies this only to a source that would create a new
    static track; an established static track remains available to the planner.
    """
    if track_length <= 0.0:
        return False
    for witness in dynamic_witnesses:
        ds = ((float(static_s) - float(witness.s) + track_length / 2.0)
              % track_length - track_length / 2.0)
        if (-behind_limit <= ds <= ahead_limit
                and abs(float(static_d) - float(witness.d)) <= lateral_limit):
            return True
    return False


def static_source_is_new(source_id, known_track_ids,
                         reassociated_track_id=None):
    """Whether a static detector source would create a new bridge track.

    A new detector ID can already be reassociated to a mature static bridge
    track.  It is not "new" for the H2H quarantine in that case: suppressing
    it would hide a confirmed cone merely because the detector renewed its ID.
    """
    source_id = int(source_id)
    if source_id in known_track_ids:
        return False
    return (reassociated_track_id is None
            or int(reassociated_track_id) not in known_track_ids)


def bridge_output_is_static(raw_is_dynamic, unsettled_class,
                            force_low_confidence_dynamic_static):
    """Return the bridge classification without changing raw provenance.

    The legacy conservative mode may present a low-confidence dynamic input as
    static.  H2H profiles can disable that conversion so a measured opponent
    stays dynamic through its classifier-confidence reset.
    """
    return (not bool(raw_is_dynamic)
            or (bool(unsettled_class)
                and bool(force_low_confidence_dynamic_static)))


class DynamicShadowQuarantine:
    """Short lease that prevents a new static source from reappearing mid-pass.

    A source is eligible only while it has no bridge track of its own.  Once a
    static obstacle has been established, a nearby opponent must not erase it.
    The lease is refreshed exclusively by current-frame dynamic witnesses, so
    it expires soon after the head-to-head interaction ends.
    """

    def __init__(self, hold_sec):
        self.hold_sec = float(hold_sec)
        self._until = {}

    def should_quarantine(self, source_id, now_sec, is_new_source,
                          is_in_dynamic_shadow):
        source_id = int(source_id)
        now_sec = float(now_sec)
        if not is_new_source:
            self._until.pop(source_id, None)
            return False
        if is_in_dynamic_shadow:
            self._until[source_id] = now_sec + self.hold_sec
            return True
        until = self._until.get(source_id)
        if until is not None and now_sec <= until:
            return True
        self._until.pop(source_id, None)
        return False

    def forget(self, source_id):
        self._until.pop(int(source_id), None)

    def is_active(self, source_id, now_sec):
        """Return whether a source still has an unexpired shadow lease."""
        source_id = int(source_id)
        until = self._until.get(source_id)
        if until is None:
            return False
        if float(now_sec) > until:
            del self._until[source_id]
            return False
        return True

    def prune(self, now_sec):
        now_sec = float(now_sec)
        for source_id, until in list(self._until.items()):
            if now_sec > until:
                del self._until[source_id]


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

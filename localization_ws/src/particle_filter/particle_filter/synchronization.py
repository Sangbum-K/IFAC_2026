# Copyright 2026 IFAC F1TENTH Team
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""Small, thread-safe buffers used to synchronize scans and odometry."""

import math
import time
from bisect import bisect_left
from collections import deque
from dataclasses import dataclass
from threading import Condition
from typing import Deque, Generic, Optional, Tuple, TypeVar


LOOKUP_OK = 'ok'
LOOKUP_TOO_OLD = 'too_old'
LOOKUP_IN_FUTURE = 'in_future'
LOOKUP_GAP_TOO_LARGE = 'gap_too_large'
LOOKUP_CLOSED = 'closed'


@dataclass(frozen=True)
class OdomSample:
    """A lightweight planar odometry sample with an integer ROS timestamp."""

    stamp_ns: int
    x: float
    y: float
    yaw: float
    linear_speed: float = 0.0
    angular_speed: float = 0.0


@dataclass(frozen=True)
class OdomLookup:
    """Result returned by an odometry interpolation request."""

    status: str
    sample: Optional[OdomSample] = None


class TimestampedOdomBuffer:
    """
    Keep a bounded timestamp-ordered odometry history.

    The condition lock is held only while appending, copying at most 150
    timestamps, or interpolating two scalar samples. No particle-filter work
    runs while this lock is held.
    """

    def __init__(self, capacity: int, reset_threshold_sec: float = 1.0):
        """Create a buffer and configure backward-time reset handling."""
        if capacity < 2:
            raise ValueError('capacity must be at least 2')
        if reset_threshold_sec <= 0.0:
            raise ValueError('reset_threshold_sec must be positive')

        self._samples: Deque[OdomSample] = deque(maxlen=capacity)
        self._condition = Condition()
        self._closed = False
        self._reset_threshold_ns = int(reset_threshold_sec * 1e9)
        self.rejected_out_of_order = 0
        self.time_reset_count = 0

    @property
    def capacity(self) -> int:
        """Return the fixed maximum number of retained samples."""
        return self._samples.maxlen

    def __len__(self) -> int:
        """Return the current sample count."""
        with self._condition:
            return len(self._samples)

    def append(self, sample: OdomSample) -> bool:
        """Append a monotonic sample, replacing an equal-timestamp sample."""
        with self._condition:
            if self._closed:
                return False

            if self._samples:
                newest_stamp = self._samples[-1].stamp_ns
                if sample.stamp_ns == newest_stamp:
                    self._samples[-1] = sample
                    self._condition.notify_all()
                    return True
                if sample.stamp_ns < newest_stamp:
                    backward_jump = newest_stamp - sample.stamp_ns
                    if backward_jump >= self._reset_threshold_ns:
                        self._samples.clear()
                        self.time_reset_count += 1
                    else:
                        self.rejected_out_of_order += 1
                        return False

            self._samples.append(sample)
            self._condition.notify_all()
            return True

    def latest(self) -> Optional[OdomSample]:
        """Return the newest sample, if available."""
        with self._condition:
            return self._samples[-1] if self._samples else None

    def clear(self) -> None:
        """Clear all history while preserving configured capacity."""
        with self._condition:
            self._samples.clear()
            self._condition.notify_all()

    def close(self) -> None:
        """Wake waiters and reject future appends."""
        with self._condition:
            self._closed = True
            self._condition.notify_all()

    def lookup(self, stamp_ns: int, max_gap_sec: float) -> OdomLookup:
        """Interpolate immediately without waiting for a future bracket."""
        with self._condition:
            return self._lookup_locked(stamp_ns, int(max_gap_sec * 1e9))

    def wait_for_interpolated(
            self, stamp_ns: int, timeout_sec: float,
            max_gap_sec: float) -> OdomLookup:
        """Wait briefly for an odometry bracket, then interpolate."""
        max_gap_ns = int(max_gap_sec * 1e9)
        deadline = time.monotonic() + max(0.0, timeout_sec)

        with self._condition:
            while True:
                result = self._lookup_locked(stamp_ns, max_gap_ns)
                if result.status != LOOKUP_IN_FUTURE:
                    return result

                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return result
                self._condition.wait(remaining)

    def _lookup_locked(self, stamp_ns: int, max_gap_ns: int) -> OdomLookup:
        if self._closed:
            return OdomLookup(LOOKUP_CLOSED)
        if not self._samples:
            return OdomLookup(LOOKUP_IN_FUTURE)

        oldest = self._samples[0]
        newest = self._samples[-1]
        if stamp_ns < oldest.stamp_ns:
            return OdomLookup(LOOKUP_TOO_OLD)
        if stamp_ns > newest.stamp_ns:
            return OdomLookup(LOOKUP_IN_FUTURE)
        if stamp_ns == oldest.stamp_ns:
            return OdomLookup(LOOKUP_OK, oldest)
        if stamp_ns == newest.stamp_ns:
            return OdomLookup(LOOKUP_OK, newest)

        samples = tuple(self._samples)
        stamps = [sample.stamp_ns for sample in samples]
        upper_index = bisect_left(stamps, stamp_ns)
        lower = samples[upper_index - 1]
        upper = samples[upper_index]
        gap_ns = upper.stamp_ns - lower.stamp_ns
        if gap_ns <= 0 or gap_ns > max_gap_ns:
            return OdomLookup(LOOKUP_GAP_TOO_LARGE)

        ratio = (stamp_ns - lower.stamp_ns) / float(gap_ns)
        yaw_delta = _wrap_angle(upper.yaw - lower.yaw)
        sample = OdomSample(
            stamp_ns=stamp_ns,
            x=_lerp(lower.x, upper.x, ratio),
            y=_lerp(lower.y, upper.y, ratio),
            yaw=_wrap_angle(lower.yaw + ratio * yaw_delta),
            linear_speed=_lerp(
                lower.linear_speed, upper.linear_speed, ratio),
            angular_speed=_lerp(
                lower.angular_speed, upper.angular_speed, ratio),
        )
        return OdomLookup(LOOKUP_OK, sample)


T = TypeVar('T')


class LatestValueSlot(Generic[T]):
    """A one-element queue that overwrites stale pending work."""

    def __init__(self):
        """Create an empty slot."""
        self._condition = Condition()
        self._value: Optional[T] = None
        self._closed = False
        self.offered_count = 0
        self.overwritten_count = 0
        self.taken_count = 0

    def offer(self, value: T) -> bool:
        """Store ``value``, replacing any pending work."""
        with self._condition:
            if self._closed:
                return False
            self.offered_count += 1
            if self._value is not None:
                self.overwritten_count += 1
            self._value = value
            self._condition.notify()
            return True

    def take(self, timeout_sec: Optional[float] = None) -> Optional[T]:
        """Take the pending value, waiting up to ``timeout_sec`` if empty."""
        deadline = None
        if timeout_sec is not None:
            deadline = time.monotonic() + max(0.0, timeout_sec)

        with self._condition:
            while self._value is None and not self._closed:
                if deadline is None:
                    self._condition.wait()
                    continue
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(remaining)

            if self._value is None:
                return None
            value = self._value
            self._value = None
            self.taken_count += 1
            return value

    def counts(self) -> Tuple[int, int, int]:
        """Return offered, overwritten, and taken counters atomically."""
        with self._condition:
            return (
                self.offered_count,
                self.overwritten_count,
                self.taken_count,
            )

    def close(self) -> None:
        """Wake waiters and reject future values."""
        with self._condition:
            self._closed = True
            self._condition.notify_all()


def _lerp(lhs: float, rhs: float, ratio: float) -> float:
    return lhs + ratio * (rhs - lhs)


def _wrap_angle(theta: float) -> float:
    return math.atan2(math.sin(theta), math.cos(theta))

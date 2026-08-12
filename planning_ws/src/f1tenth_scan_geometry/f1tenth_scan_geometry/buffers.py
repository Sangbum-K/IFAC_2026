"""Small bounded timestamp buffers with vectorized interpolation."""

from collections import deque
from dataclasses import dataclass
from threading import Condition
import time

import numpy as np

from .geometry import slerp_quaternions


LOOKUP_OK = 'ok'
LOOKUP_TOO_OLD = 'too_old'
LOOKUP_IN_FUTURE = 'in_future'
LOOKUP_GAP_TOO_LARGE = 'gap_too_large'
LOOKUP_CLOSED = 'closed'


@dataclass(frozen=True)
class PoseSample:
    stamp_ns: int
    x: float
    y: float
    yaw: float
    linear_speed: float = 0.0


@dataclass(frozen=True)
class AttitudeSample:
    stamp_ns: int
    quaternion: tuple
    angular_speed: float = 0.0
    acceleration_norm: float = 0.0


@dataclass(frozen=True)
class PoseBatchLookup:
    status: str
    samples: tuple = ()


@dataclass(frozen=True)
class AttitudeBatchLookup:
    status: str
    quaternions: object = None


def _wrap_angle(value):
    return np.arctan2(np.sin(value), np.cos(value))


class _TimestampedBuffer:
    def __init__(self, capacity, reset_threshold_sec):
        if capacity < 2:
            raise ValueError('capacity must be at least two')
        if reset_threshold_sec <= 0.0:
            raise ValueError('reset_threshold_sec must be positive')
        self._samples = deque(maxlen=int(capacity))
        self._condition = Condition()
        self._closed = False
        self._reset_threshold_ns = int(reset_threshold_sec * 1e9)
        self.rejected_out_of_order = 0
        self.time_reset_count = 0

    def __len__(self):
        with self._condition:
            return len(self._samples)

    def append(self, sample):
        with self._condition:
            if self._closed:
                return False
            if self._samples:
                newest = self._samples[-1].stamp_ns
                if sample.stamp_ns == newest:
                    self._samples[-1] = sample
                    self._condition.notify_all()
                    return True
                if sample.stamp_ns < newest:
                    if newest - sample.stamp_ns >= self._reset_threshold_ns:
                        self._samples.clear()
                        self.time_reset_count += 1
                    else:
                        self.rejected_out_of_order += 1
                        return False
            self._samples.append(sample)
            self._condition.notify_all()
            return True

    def latest(self):
        with self._condition:
            return self._samples[-1] if self._samples else None

    def clear(self):
        with self._condition:
            self._samples.clear()
            self._condition.notify_all()

    def close(self):
        with self._condition:
            self._closed = True
            self._condition.notify_all()

    def _snapshot(self):
        with self._condition:
            return self._closed, tuple(self._samples)

    def _wait_snapshot(self, newest_stamp_ns, timeout_sec):
        deadline = time.monotonic() + max(0.0, timeout_sec)
        with self._condition:
            while (not self._closed
                   and (not self._samples
                        or self._samples[-1].stamp_ns < newest_stamp_ns)):
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._condition.wait(remaining)
            return self._closed, tuple(self._samples)


def _brackets(samples, requested, max_gap_ns):
    if not requested:
        return LOOKUP_OK, None, None, None
    if not samples:
        return LOOKUP_IN_FUTURE, None, None, None
    stamps = np.fromiter((s.stamp_ns for s in samples), dtype=np.int64)
    target = np.asarray(requested, dtype=np.int64)
    if target.min() < stamps[0]:
        return LOOKUP_TOO_OLD, None, None, None
    if target.max() > stamps[-1]:
        return LOOKUP_IN_FUTURE, None, None, None
    upper = np.searchsorted(stamps, target, side='left')
    exact = stamps[np.minimum(upper, len(stamps) - 1)] == target
    lower = np.where(exact, upper, upper - 1)
    upper = np.where(exact, upper, upper)
    gaps = stamps[upper] - stamps[lower]
    if np.any((~exact) & ((gaps <= 0) | (gaps > max_gap_ns))):
        return LOOKUP_GAP_TOO_LARGE, None, None, None
    ratios = np.zeros(len(target), dtype=np.float64)
    moving = ~exact
    ratios[moving] = ((target[moving] - stamps[lower[moving]])
                      / gaps[moving].astype(np.float64))
    return LOOKUP_OK, lower, upper, ratios


class TimestampedPoseBuffer(_TimestampedBuffer):
    def __init__(self, capacity, reset_threshold_sec=1.0):
        super().__init__(capacity, reset_threshold_sec)

    def lookup_many(self, stamps_ns, max_gap_sec):
        closed, samples = self._snapshot()
        return self._interpolate(closed, samples, stamps_ns, max_gap_sec)

    def wait_for_lookup_many(self, stamps_ns, timeout_sec, max_gap_sec):
        requested = tuple(int(v) for v in stamps_ns)
        newest = max(requested) if requested else 0
        closed, samples = self._wait_snapshot(newest, timeout_sec)
        return self._interpolate(closed, samples, requested, max_gap_sec)

    @staticmethod
    def _interpolate(closed, samples, stamps_ns, max_gap_sec):
        if closed:
            return PoseBatchLookup(LOOKUP_CLOSED)
        requested = tuple(int(v) for v in stamps_ns)
        status, lower, upper, ratios = _brackets(
            samples, requested, int(max_gap_sec * 1e9))
        if status != LOOKUP_OK:
            return PoseBatchLookup(status)
        if not requested:
            return PoseBatchLookup(LOOKUP_OK)
        values = np.array([[s.x, s.y, s.yaw, s.linear_speed]
                           for s in samples], dtype=np.float64)
        lo = values[lower]
        hi = values[upper]
        out = lo + ratios[:, None] * (hi - lo)
        yaw_delta = _wrap_angle(hi[:, 2] - lo[:, 2])
        out[:, 2] = _wrap_angle(lo[:, 2] + ratios * yaw_delta)
        result = tuple(PoseSample(stamp, *row) for stamp, row
                       in zip(requested, out))
        return PoseBatchLookup(LOOKUP_OK, result)


class TimestampedAttitudeBuffer(_TimestampedBuffer):
    def __init__(self, capacity, reset_threshold_sec=1.0):
        super().__init__(capacity, reset_threshold_sec)

    def lookup_many(self, stamps_ns, max_gap_sec):
        closed, samples = self._snapshot()
        return self._interpolate(closed, samples, stamps_ns, max_gap_sec)

    def wait_for_lookup_many(self, stamps_ns, timeout_sec, max_gap_sec):
        requested = tuple(int(v) for v in stamps_ns)
        newest = max(requested) if requested else 0
        closed, samples = self._wait_snapshot(newest, timeout_sec)
        return self._interpolate(closed, samples, requested, max_gap_sec)

    @staticmethod
    def _interpolate(closed, samples, stamps_ns, max_gap_sec):
        if closed:
            return AttitudeBatchLookup(LOOKUP_CLOSED)
        requested = tuple(int(v) for v in stamps_ns)
        status, lower, upper, ratios = _brackets(
            samples, requested, int(max_gap_sec * 1e9))
        if status != LOOKUP_OK:
            return AttitudeBatchLookup(status)
        if not requested:
            return AttitudeBatchLookup(
                LOOKUP_OK, np.empty((0, 4), dtype=np.float64))
        quaternions = np.asarray(
            [s.quaternion for s in samples], dtype=np.float64)
        output = slerp_quaternions(
            quaternions[lower], quaternions[upper], ratios)
        return AttitudeBatchLookup(LOOKUP_OK, output)

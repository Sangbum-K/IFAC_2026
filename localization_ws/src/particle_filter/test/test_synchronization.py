# Copyright 2026 IFAC F1TENTH Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for timestamp synchronization primitives."""

import math
import time
from threading import Thread

from particle_filter.synchronization import LOOKUP_GAP_TOO_LARGE
from particle_filter.synchronization import LOOKUP_OK
from particle_filter.synchronization import LatestValueSlot
from particle_filter.synchronization import OdomSample
from particle_filter.synchronization import TimestampedOdomBuffer


def test_odom_buffer_is_bounded_and_rejects_small_backward_jump():
    """The history stays bounded without accepting stale packets."""
    buffer = TimestampedOdomBuffer(capacity=3)
    for stamp in range(5):
        sample = OdomSample(
            stamp_ns=stamp * 1_000_000, x=stamp, y=0, yaw=0)
        assert buffer.append(sample)

    assert len(buffer) == 3
    assert not buffer.append(OdomSample(stamp_ns=3_500_000, x=0, y=0, yaw=0))
    assert buffer.rejected_out_of_order == 1


def test_interpolation_uses_shortest_yaw_path():
    """Interpolation across the angle branch cut must stay near 180 degrees."""
    buffer = TimestampedOdomBuffer(capacity=10)
    buffer.append(OdomSample(
        stamp_ns=1_000_000_000, x=0.0, y=1.0, yaw=math.radians(179.0)))
    buffer.append(OdomSample(
        stamp_ns=1_020_000_000, x=2.0, y=3.0, yaw=math.radians(-179.0)))

    result = buffer.lookup(1_010_000_000, max_gap_sec=0.05)

    assert result.status == LOOKUP_OK
    assert result.sample is not None
    assert math.isclose(result.sample.x, 1.0)
    assert math.isclose(result.sample.y, 2.0)
    assert math.isclose(abs(result.sample.yaw), math.pi, abs_tol=1e-9)


def test_interpolation_rejects_an_odom_gap_that_is_too_large():
    """A missing odometry interval must not be hidden by interpolation."""
    buffer = TimestampedOdomBuffer(capacity=10)
    buffer.append(OdomSample(stamp_ns=1_000_000_000, x=0, y=0, yaw=0))
    buffer.append(OdomSample(stamp_ns=1_100_000_000, x=1, y=0, yaw=0))

    result = buffer.lookup(1_050_000_000, max_gap_sec=0.06)

    assert result.status == LOOKUP_GAP_TOO_LARGE
    assert result.sample is None


def test_large_backward_time_jump_starts_a_new_history():
    """A bag loop or clock reset must not permanently freeze odom input."""
    buffer = TimestampedOdomBuffer(capacity=10)
    buffer.append(OdomSample(stamp_ns=2_000_000_000, x=0, y=0, yaw=0))
    buffer.append(OdomSample(stamp_ns=3_000_000_000, x=1, y=0, yaw=0))

    assert buffer.append(OdomSample(
        stamp_ns=1_000_000_000, x=2, y=0, yaw=0))
    assert len(buffer) == 1
    assert buffer.time_reset_count == 1
    assert buffer.latest().stamp_ns == 1_000_000_000


def test_wait_for_interpolation_accepts_delayed_future_bracket():
    """A scan may wait briefly for the next odometry sample to arrive."""
    buffer = TimestampedOdomBuffer(capacity=10)
    buffer.append(OdomSample(stamp_ns=1_000_000_000, x=0.0, y=0.0, yaw=0.0))

    def append_future_sample():
        time.sleep(0.01)
        buffer.append(OdomSample(
            stamp_ns=1_020_000_000, x=0.2, y=0.0, yaw=0.02))

    thread = Thread(target=append_future_sample)
    thread.start()
    result = buffer.wait_for_interpolated(
        1_010_000_000, timeout_sec=0.05, max_gap_sec=0.05)
    thread.join()

    assert result.status == LOOKUP_OK
    assert result.sample is not None
    assert math.isclose(result.sample.x, 0.1)
    assert math.isclose(result.sample.yaw, 0.01)


def test_latest_value_slot_overwrites_pending_work():
    """Only the newest unprocessed scan-like value is retained."""
    slot = LatestValueSlot[int]()

    assert slot.offer(1)
    assert slot.offer(2)
    assert slot.offer(3)
    assert slot.take(timeout_sec=0.0) == 3
    assert slot.take(timeout_sec=0.0) is None
    assert slot.counts() == (3, 2, 1)

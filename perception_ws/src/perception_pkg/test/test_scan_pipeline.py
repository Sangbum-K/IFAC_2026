import numpy as np

from perception_pkg.clustering import adaptive_cluster_points
from perception_pkg.clustering import adaptive_component_provenance
from perception_pkg.preprocessing import preprocess_scan
from perception_pkg.preprocessing import sanitize_scan_ranges


def test_ust_no_return_is_invalid_before_median():
    points, ranges, indices, _angles = preprocess_scan(
        [1.0, 65.533, 65.533, 2.0, 2.0], -0.2, 0.1,
        range_min=0.1, range_max=12.0, sensor_range_max=30.0,
        median_kernel=3, return_metadata=True)
    assert np.array_equal(indices, [0, 3, 4])
    assert np.allclose(ranges, [1.0, 2.0, 2.0])
    assert len(points) == 3


def test_sanitized_ranges_preserve_original_beam_positions():
    ranges, angles = sanitize_scan_ranges(
        [1.0, 65.533, np.inf, 2.0], -0.2, 0.1,
        range_min=0.1, range_max=12.0, sensor_range_max=30.0)
    assert np.array_equal(np.flatnonzero(np.isfinite(ranges)), [0, 3])
    assert np.allclose(angles, [-0.2, -0.1, 0.0, 0.1])


def test_filtered_wall_fragments_keep_large_source_radius():
    angles = np.linspace(-0.2, 0.2, 21)
    ranges = np.full(21, 3.0)
    points = np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))
    indices = np.arange(21)
    labels, radii, sizes = adaptive_component_provenance(
        points, ranges, 0.02, lambda_angle=np.deg2rad(10.0),
        beam_indices=indices)
    keep = np.r_[np.arange(5), np.arange(16, 21)]
    clusters = adaptive_cluster_points(
        points[keep], ranges[keep], 0.02, lambda_angle=np.deg2rad(10.0),
        min_cluster_size=3, beam_indices=indices[keep],
        source_labels=labels[keep], source_radii=radii, source_sizes=sizes)
    assert len(clusters) == 2
    assert radii[0] > 0.5
    assert all(np.isclose(cluster.source_radius, radii[0]) for cluster in clusters)
    assert all(cluster.source_size == 21 for cluster in clusters)


def test_removed_beam_gap_does_not_use_fixed_angle_increment():
    points = np.array([[1.0, 0.0], [0.0, 1.0]])
    ranges = np.ones(2)
    clusters = adaptive_cluster_points(
        points, ranges, np.deg2rad(1.0), lambda_angle=np.deg2rad(10.0),
        min_cluster_size=1, beam_indices=np.array([0, 90]))
    assert len(clusters) == 2

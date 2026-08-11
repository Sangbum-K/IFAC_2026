"""Regression tests for the post-filter obstacle radius gate."""

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from perception_pkg.clustering import adaptive_cluster_points
from perception_pkg.clustering import adaptive_component_provenance
from perception_pkg.obstacle_detector_node import (
    partition_clusters_by_postfilter_radius,
)


def test_run_0809_231809_61_2_keeps_compact_postfilter_obstacle():
    """Rebuild the exact late-acquisition cluster at record elapsed 61.2 s."""
    fixture_path = Path(__file__).with_name(
        'run_0809_231809_61_2_radius_gate.json')
    frame = json.loads(fixture_path.read_text(encoding='utf-8'))

    assert frame['elapsed_time_basis'] == 'rosbag_record_timestamp'
    elapsed = (
        frame['scan_record_timestamp_ns'] - frame['bag_start_record_ns']
    ) / 1.0e9
    assert np.isclose(elapsed, frame['record_elapsed_sec'], atol=1.0e-12)
    assert round(elapsed, 1) == 61.2

    points = np.asarray(frame['source_points_xy_m_map'], dtype=np.float64)
    ranges = np.asarray(frame['source_laser_ranges_m'], dtype=np.float64)
    beam_indices = np.asarray(frame['source_beam_indices'], dtype=np.int64)
    wall_distances = np.asarray(frame['wall_distance_m'], dtype=np.float64)
    labels, source_radii, source_sizes = adaptive_component_provenance(
        points,
        ranges,
        frame['angle_increment_rad'],
        lambda_angle=frame['lambda_angle_rad'],
        sigma=frame['sigma_m'],
        beam_indices=beam_indices,
    )
    assert len(source_radii) == 1
    assert labels[0] == frame['compact_source_component_zero_based']
    assert np.all(labels == labels[0])
    assert source_sizes[0] == frame['source_size']
    assert np.isclose(
        source_radii[0], frame['source_radius_m'], atol=1.0e-12)

    keep_mask = wall_distances >= frame['wall_dist_min_m']
    assert np.array_equal(
        beam_indices[keep_mask], frame['retained_beam_indices'])
    clusters = adaptive_cluster_points(
        points[keep_mask],
        ranges[keep_mask],
        frame['angle_increment_rad'],
        lambda_angle=frame['lambda_angle_rad'],
        sigma=frame['sigma_m'],
        min_cluster_size=frame['min_cluster_size'],
        beam_indices=beam_indices[keep_mask],
        source_labels=labels[keep_mask],
        source_radii=source_radii,
        source_sizes=source_sizes,
    )

    assert len(clusters) == 1
    cluster = clusters[0]
    assert len(cluster.points) == frame['retained_size']
    assert np.allclose(
        cluster.centroid, frame['post_filter_centroid_xy_m'], atol=1.0e-12)
    assert np.isclose(
        cluster.radius, frame['post_filter_radius_m'], atol=1.0e-12)
    assert np.isclose(
        cluster.source_radius, frame['source_radius_m'], atol=1.0e-12)
    assert cluster.source_size == frame['source_size']

    assert cluster.radius < frame['max_cluster_radius_m']
    assert cluster.source_radius > frame['max_cluster_radius_m']

    kept, rejected = partition_clusters_by_postfilter_radius(
        [cluster], frame['max_cluster_radius_m'])

    assert frame['expected'] == 'keep'
    assert kept == [cluster]
    assert rejected == []


def test_radius_gate_still_rejects_oversized_postfilter_geometry():
    """Keep source provenance without allowing a physically large result."""
    compact = SimpleNamespace(radius=0.22, source_radius=0.54)
    oversized = SimpleNamespace(radius=0.36, source_radius=0.36)

    kept, rejected = partition_clusters_by_postfilter_radius(
        [compact, oversized], max_radius=0.35)

    assert kept == [compact]
    assert rejected == [oversized]

#!/usr/bin/env python3
"""Serve the exact OccupancyGrid stored in a bag through nav_msgs/GetMap.

The PF fetches its map through ``/map_server/map`` during construction.  This
small replay-only server avoids converting the recorded map to an image/YAML
pair (which can change unknown/occupied cells) and keeps the A/B input exact.
It does not publish or control any vehicle topic.
"""

import argparse
import copy
import math
from pathlib import Path

from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message

import rosbag2_py

from tf2_msgs.msg import TFMessage

from tf2_ros import StaticTransformBroadcaster


def storage_id(path):
    """Infer the rosbag2 storage plugin used by a bag directory or file."""
    path = Path(path)
    if path.suffix == '.db3':
        return 'sqlite3'
    if path.suffix == '.mcap':
        return 'mcap'
    metadata = path / 'metadata.yaml'
    if metadata.is_file():
        text = metadata.read_text(encoding='utf-8')
        for line in text.splitlines():
            if line.strip().startswith('storage_identifier:'):
                return line.split(':', 1)[1].strip().strip("'\"")
    if any(path.glob('*.db3')):
        return 'sqlite3'
    if any(path.glob('*.mcap')):
        return 'mcap'
    raise RuntimeError('Cannot infer rosbag storage for %s' % path)


def load_recorded_map(path):
    """Read and deserialize the first map without replaying the bag."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(Path(path).resolve()), storage_id=storage_id(path)),
        rosbag2_py.ConverterOptions('', ''),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    actual_type = topic_types.get('/map')
    expected_type = 'nav_msgs/msg/OccupancyGrid'
    if actual_type != expected_type:
        raise RuntimeError(
            'Bag /map type is %r; expected %s' % (actual_type, expected_type))
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic == '/map':
            return deserialize_message(serialized, OccupancyGrid)
    raise RuntimeError('Bag contains no /map message: %s' % path)


def normalized_frame(frame):
    """Normalize optional leading slashes for TF frame comparisons."""
    return frame.strip().lstrip('/')


def load_recorded_static_transforms(path):
    """Read the last recorded value for every static TF edge."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(Path(path).resolve()), storage_id=storage_id(path)),
        rosbag2_py.ConverterOptions('', ''),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    actual_type = topic_types.get('/tf_static')
    expected_type = 'tf2_msgs/msg/TFMessage'
    if actual_type != expected_type:
        raise RuntimeError(
            'Bag /tf_static type is %r; expected %s'
            % (actual_type, expected_type))

    transforms = {}
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic != '/tf_static':
            continue
        message = deserialize_message(serialized, TFMessage)
        for transform in message.transforms:
            key = (
                normalized_frame(transform.header.frame_id),
                normalized_frame(transform.child_frame_id),
            )
            transforms[key] = transform
    if not transforms:
        raise RuntimeError('Bag contains no static transforms: %s' % path)
    return list(transforms.values())


def find_direct_static_transform(
        transforms, parent_frame='base_link', child_frame='laser'):
    """Return the unique direct static transform for one TF edge."""
    parent_frame = normalized_frame(parent_frame)
    child_frame = normalized_frame(child_frame)
    matches = [
        transform for transform in transforms
        if normalized_frame(transform.header.frame_id) == parent_frame
        and normalized_frame(transform.child_frame_id) == child_frame
    ]
    if len(matches) != 1:
        raise RuntimeError(
            'Expected exactly one recorded %s -> %s static TF; found %d'
            % (parent_frame, child_frame, len(matches)))
    return matches[0]


def override_base_to_laser_z(transforms, height_m):
    """Copy recorded static TFs and replace only base_link -> laser z."""
    if not math.isfinite(height_m):
        raise ValueError('base_link -> laser z override must be finite')
    modified = copy.deepcopy(transforms)
    transform = find_direct_static_transform(modified)
    original_height_m = float(transform.transform.translation.z)
    transform.transform.translation.z = float(height_m)
    return modified, original_height_m


class RecordedMapServer(Node):
    """Provide the recorded map and an optional calibrated static TF set."""

    def __init__(
            self, map_message, bag_path, static_transforms=None,
            base_to_laser_z_m=None):
        """Expose immutable replay inputs through the interfaces used by PF."""
        super().__init__('pf_replay_map_server')
        self._map_message = map_message
        self._service = self.create_service(
            GetMap, '/map_server/map', self._get_map)
        self._static_broadcaster = None
        if base_to_laser_z_m is not None:
            if static_transforms is None:
                raise RuntimeError(
                    'Static TF messages are required for a LiDAR z override')
            modified, original_height_m = override_base_to_laser_z(
                static_transforms, base_to_laser_z_m)
            self._static_broadcaster = StaticTransformBroadcaster(self)
            self._static_broadcaster.sendTransform(modified)
            self.get_logger().info(
                'Publishing recorded /tf_static with base_link -> laser '
                'z override: %.4f m -> %.4f m (%d transforms)'
                % (
                    original_height_m,
                    base_to_laser_z_m,
                    len(modified),
                ))
        self.get_logger().info(
            'Serving recorded /map from %s (%dx%d, %.4f m/cell)'
            % (
                bag_path,
                map_message.info.width,
                map_message.info.height,
                map_message.info.resolution,
            ))

    def _get_map(self, _request, response):
        response.map = self._map_message
        return response


def main():
    """Load the bag map and serve it until the orchestrator stops us."""
    parser = argparse.ArgumentParser(
        description=(
            'Serve a bag OccupancyGrid at /map_server/map for PF replay'))
    parser.add_argument('bag', help='rosbag2 directory, .db3, or .mcap')
    parser.add_argument(
        '--base-to-laser-z-m', type=float,
        help=(
            'replace only the recorded base_link -> laser static-TF z; '
            'also republishes every other recorded static transform'))
    args = parser.parse_args()

    bag_path = Path(args.bag).expanduser().resolve()
    map_message = load_recorded_map(bag_path)
    static_transforms = None
    if args.base_to_laser_z_m is not None:
        static_transforms = load_recorded_static_transforms(bag_path)
    rclpy.init()
    node = RecordedMapServer(
        map_message,
        bag_path,
        static_transforms=static_transforms,
        base_to_laser_z_m=args.base_to_laser_z_m,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # SIGINT may already have shut the default context down through
        # rclpy's signal handler.  Avoid turning a clean orchestrator stop into
        # a misleading RCLError in the replay log.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

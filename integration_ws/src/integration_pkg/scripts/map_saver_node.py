#!/usr/bin/env python3
"""slam_toolbox가 만드는 /map을 계속 캐시했다가 파일로 떨구는 노드.

mapping.launch.py가 띄우며, 저장 시점은 두 가지다:

  1. Ctrl+C (종료)      — 캐시된 마지막 /map을 pgm + yaml로 저장 (기본 동작)
  2. /save_map 서비스   — 주행 중 아무 때나 중간 저장
        ros2 service call /save_map std_srvs/srv/Trigger

핵심은 "/map을 미리 캐시해 둔다"는 점이다. 종료 시점에 slam_toolbox에
서비스를 호출하는 방식은, launch가 SIGINT를 프로세스 그룹 전체에 동시에
보내기 때문에 slam_toolbox가 이미 죽는 중이라 자주 실패한다. 캐시본을
직접 쓰면 ROS 통신이 필요 없어 항상 성공한다.

출력 형식은 nav2 map_server / particle_filter가 읽는 표준 포맷이며,
particle_filter/maps/my_map.yaml과 동일한 키 구성을 따른다.
"""

import math
import os

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger

# nav2 map_server 기본 임계값. yaml에도 같은 값을 기록해 로드 시 일치시킨다.
OCCUPIED_THRESH = 0.65
FREE_THRESH = 0.25

# PGM 픽셀 값 (map_saver_cli 관례)
PIX_OCCUPIED = 0      # 검정 = 벽
PIX_FREE = 254        # 흰색 = 주행 가능
PIX_UNKNOWN = 205     # 회색 = 미탐사


class MapSaverNode(Node):

    def __init__(self):
        super().__init__('map_saver_node')

        # mapping.launch.py 는 map_dir 을 항상 명시적으로 넘기므로 이 기본값은
        # ros2 run 으로 단독 실행할 때만 쓰인다. launch 기본값과 같은 곳을
        # 가리켜 두 경로가 어긋나 보이지 않게 한다.
        self.declare_parameter('map_dir', os.path.expanduser('~/IFAC_ws/maps'))
        self.declare_parameter('map_name', 'my_map')
        self.declare_parameter('map_topic', '/map')
        # 종료(Ctrl+C) 시 자동 저장 여부. false면 /save_map 서비스로만 저장한다.
        self.declare_parameter('save_on_exit', True)

        self.map_dir = os.path.expanduser(self.get_parameter('map_dir').value)
        self.map_name = self.get_parameter('map_name').value
        self.save_on_exit = bool(self.get_parameter('save_on_exit').value)

        self.grid = None      # 마지막으로 받은 OccupancyGrid
        self.saved = False    # 중복 저장 방지 (서비스 저장 후 종료 등)

        # slam_toolbox의 /map은 transient_local(latched) + reliable로 나간다.
        # QoS가 어긋나면 구독은 되는데 메시지가 한 개도 안 오므로 반드시 맞춘다.
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value,
            self.map_cb, map_qos)

        self.create_service(Trigger, '/save_map', self.save_service_cb)

        self.get_logger().info(
            '맵 저장 대기 중 — 저장 위치: %s'
            % os.path.join(self.map_dir, self.map_name + '.{pgm,yaml}'))
        self.get_logger().info(
            '주행이 끝나면 Ctrl+C — 그 시점의 맵이 자동 저장됩니다. '
            '중간 저장은 "ros2 service call /save_map std_srvs/srv/Trigger".')

    def map_cb(self, msg):
        first = self.grid is None
        self.grid = msg
        if first:
            self.get_logger().info(
                '첫 /map 수신 (%dx%d, %.3f m/px) — slam_toolbox 정상 동작 중'
                % (msg.info.width, msg.info.height, msg.info.resolution))

    def save_service_cb(self, _request, response):
        ok, message = self.save()
        response.success = ok
        response.message = message
        return response

    def save(self):
        """캐시된 OccupancyGrid를 pgm + yaml로 기록. (성공여부, 메시지) 반환."""
        if self.grid is None:
            msg = ('/map을 한 번도 받지 못해 저장할 게 없습니다 '
                   '(slam_toolbox가 안 떴거나 /scan이 없음)')
            self.get_logger().error(msg)
            return False, msg

        grid = self.grid
        info = grid.info
        width, height = info.width, info.height

        try:
            os.makedirs(self.map_dir, exist_ok=True)
        except OSError as e:
            msg = '맵 디렉터리를 만들 수 없습니다 (%s): %s' % (self.map_dir, e)
            self.get_logger().error(msg)
            return False, msg

        # OccupancyGrid: -1 미탐사, 0 자유, 100 점유 (행 0이 origin = 이미지 아래쪽)
        data = np.asarray(grid.data, dtype=np.int8).reshape(height, width)
        img = np.full((height, width), PIX_UNKNOWN, dtype=np.uint8)
        img[(data >= 0) & (data <= int(FREE_THRESH * 100))] = PIX_FREE
        img[data >= int(OCCUPIED_THRESH * 100)] = PIX_OCCUPIED
        # PGM은 행 0이 이미지 위쪽이라 상하를 뒤집어야 좌표가 맞는다.
        img = np.flipud(img)

        pgm_path = os.path.join(self.map_dir, self.map_name + '.pgm')
        yaml_path = os.path.join(self.map_dir, self.map_name + '.yaml')

        q = info.origin.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # 실제 크기(파생값). map_server 는 이 키들을 읽지 않고 무시한다 —
        # 사람이 맵 규모를 바로 알아보라고 넣는 참고용이다.
        # bounds 는 yaw=0(축 정렬)을 가정한다. slam_toolbox 는 항상 yaw=0으로
        # 내보내지만, 회전된 맵이면 이 범위는 맞지 않는다.
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        size_x_m = width * info.resolution
        size_y_m = height * info.resolution

        try:
            with open(pgm_path, 'wb') as f:
                f.write(b'P5\n%d %d\n255\n' % (width, height))
                f.write(img.tobytes())
            with open(yaml_path, 'w') as f:
                f.write(
                    'image: %s\n'
                    'mode: trinary\n'
                    'resolution: %.6f\n'
                    'origin: [%.6f, %.6f, %.6f]\n'
                    'negate: 0\n'
                    'occupied_thresh: %.2f\n'
                    'free_thresh: %.2f\n'
                    '\n'
                    '# 이하 참고용 (map_server 는 읽지 않음)\n'
                    'size_px: [%d, %d]\n'
                    'size_m: [%.2f, %.2f]\n'
                    'bounds_x_m: [%.2f, %.2f]\n'
                    'bounds_y_m: [%.2f, %.2f]\n'
                    % (os.path.basename(pgm_path), info.resolution,
                       origin_x, origin_y, yaw,
                       OCCUPIED_THRESH, FREE_THRESH,
                       width, height,
                       size_x_m, size_y_m,
                       origin_x, origin_x + size_x_m,
                       origin_y, origin_y + size_y_m))
        except OSError as e:
            msg = '맵 파일 기록 실패: %s' % e
            self.get_logger().error(msg)
            return False, msg

        self.saved = True
        msg = '맵 저장 완료: %s (+ .pgm, %dx%d)' % (yaml_path, width, height)
        self.get_logger().info(msg)
        return True, msg

    def save_on_shutdown(self):
        """종료 경로에서 호출. ROS 통신 없이 캐시본만 쓰므로 항상 동작한다."""
        if not self.save_on_exit:
            return
        ok, _ = self.save()
        if ok:
            # 로거가 이미 내려갔을 수 있어 stdout에도 남긴다.
            print('\n[map_saver] 맵을 저장했습니다: %s'
                  % os.path.join(self.map_dir, self.map_name + '.yaml'))
        else:
            print('\n[map_saver] 맵 저장 실패 — 위 로그를 확인하세요.')


def main(args=None):
    rclpy.init(args=args)
    node = MapSaverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # launch가 SIGINT를 보내면 컨텍스트가 이미 shutdown된 상태로 spin이 깨진다.
        pass
    finally:
        node.save_on_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

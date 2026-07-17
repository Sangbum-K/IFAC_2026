#!/usr/bin/env python3
"""RViz raceline 시각화 노드 (UNICORN/TUM 스타일).

step4_main.py가 만든 CSV들을 읽어 RViz MarkerArray로 발행한다:
  - IQP raceline   : 주황 LINE_STRIP
  - 속도 마커      : 웨이포인트마다 수직 막대 (높이 = vx * speed_scale,
                     색 = 빠름(초록) ~ 느림(빨강)) + 꼭대기를 잇는 리본
  - Wall boundary  : 벽 폴리라인을 wall_height로 세운 반투명 3D 벽
  - Centerline     : 회색 얇은 LINE_STRIP (비교용)

인터랙티브 튜너 사용법:
  1. rqt_reconfigure(또는 ros2 param set)로 이 노드의 파라미터를 바꾸면
     마커가 즉시 다시 그려진다 (speed_scale, wall_height, show_* 등).
  2. raceline 자체를 다시 뽑고 싶으면 step4_main.py를 다른 인자로 재실행
     (--safe-dist, --v-max, --mu ...). 이 노드는 CSV 파일의 수정 시각을
     감시하다가 바뀌면 자동으로 다시 로드한다. 즉 "파이썬에서 재최적화
     -> RViz에 자동 반영" 루프가 된다.

실행 (ROS2 환경, 예: f1tenth_gym_ros 도커 안):
  python3 raceline_viz_node.py --ros-args \
      -p raceline_csv:=/sim_ws/src/planning_ws/planner_pkg/config/global_trajectory/Spielberg_map_iqp_max7.00ms_margin0.15m_mu0.85.csv \
      -p centerline_csv:=/sim_ws/src/planning_ws/planner_pkg/config/centerline/Spielberg_map_centerline.csv \
      -p walls_csv:=/sim_ws/src/planning_ws/planner_pkg/config/centerline/Spielberg_map_walls.csv
RViz에서 Add -> By topic -> /raceline_viz/markers (MarkerArray) 추가.
rqt_reconfigure 실행: ros2 run rqt_reconfigure rqt_reconfigure
"""

import os

import numpy as np


# ------------------------------------------------------------- CSV 로더
def load_raceline(path):
    """9열 세미콜론 raceline CSV -> dict(xy, vx)."""
    d = np.genfromtxt(path, delimiter=";", comments="#")
    return {"xy": d[:, 1:3], "vx": d[:, 7]}


def load_xy_csv(path):
    """x; y (선택적 추가 열) CSV -> (N,2) 또는 (N,3)."""
    return np.atleast_2d(np.genfromtxt(path, delimiter=";", comments="#"))


# ----------------------------------------- 마커 기하 (rclpy 없이 테스트 가능)
def speed_color(v, v_max):
    """속도 -> (r, g, b). 빠름=초록, 느림=빨강 (브레이킹 존이 눈에 띄게)."""
    t = float(np.clip(v / max(v_max, 1e-6), 0.0, 1.0))
    return (1.0 - t, t, 0.05)


def build_speed_bars(xy, vx, scale, stride):
    """수직 속도 막대: [(밑점, 꼭대기점, 색)] 리스트."""
    bars = []
    v_max = float(vx.max())
    for i in range(0, len(xy), max(stride, 1)):
        h = float(vx[i]) * scale
        bars.append(((xy[i, 0], xy[i, 1], 0.0),
                     (xy[i, 0], xy[i, 1], h),
                     speed_color(vx[i], v_max)))
    return bars


def build_speed_ribbon(xy, vx, scale):
    """막대 꼭대기를 잇는 리본 점열 [(x, y, z)] (닫힌 루프)."""
    pts = [(xy[i, 0], xy[i, 1], float(vx[i]) * scale)
           for i in range(len(xy))]
    return pts + [pts[0]]


def build_wall_triangles(wall_xy, height):
    """벽 폴리라인 -> 세워진 벽의 TRIANGLE_LIST 점열 [(x,y,z) x 3n].

    인접한 두 점 p0, p1로 사각 패널(p0,0)-(p1,0)-(p1,h)-(p0,h)을 만들고
    삼각형 2개로 쪼갠다. 폐곡선이므로 마지막-처음 점도 잇는다.
    """
    tris = []
    n = len(wall_xy)
    for i in range(n):
        x0, y0 = wall_xy[i]
        x1, y1 = wall_xy[(i + 1) % n]
        lo0, lo1 = (x0, y0, 0.0), (x1, y1, 0.0)
        hi0, hi1 = (x0, y0, height), (x1, y1, height)
        tris += [lo0, lo1, hi1]   # 삼각형 1
        tris += [lo0, hi1, hi0]   # 삼각형 2
    return tris


# ------------------------------------------------------------- ROS2 노드
def main():
    import rclpy
    from rclpy.node import Node
    from rcl_interfaces.msg import SetParametersResult
    from geometry_msgs.msg import Point
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import Marker, MarkerArray

    class RacelineVizNode(Node):
        def __init__(self):
            super().__init__("raceline_viz")
            # --- 파라미터 (rqt_reconfigure에서 실시간 조정 가능) ---
            self.declare_parameter("raceline_csv", "")
            self.declare_parameter("centerline_csv", "")
            self.declare_parameter("walls_csv", "")
            self.declare_parameter("frame_id", "map")
            self.declare_parameter("wall_height", 0.3)     # 벽 높이 [m]
            self.declare_parameter("wall_alpha", 0.35)     # 벽 투명도
            self.declare_parameter("speed_scale", 0.15)    # 막대높이 = vx*scale
            self.declare_parameter("speed_stride", 5)      # 막대 간격 (5=0.5m)
            self.declare_parameter("show_raceline", True)
            self.declare_parameter("show_speed", True)
            self.declare_parameter("show_walls", True)
            self.declare_parameter("show_centerline", True)

            self.pub = self.create_publisher(MarkerArray, "/raceline_viz/markers", 1)
            self._mtimes = {}
            self._data = {}
            self.add_on_set_parameters_callback(self._on_params)
            # 1 Hz: CSV 변경 감지 + (변경/파라미터 갱신 시) 재발행
            self._dirty = True
            self.create_timer(1.0, self._tick)
            self.get_logger().info("raceline_viz ready — RViz에서 "
                                   "/raceline_viz/markers (MarkerArray) 추가")

        def _on_params(self, _params):
            self._dirty = True     # 다음 tick에 다시 그림
            return SetParametersResult(successful=True)

        def _p(self, name):
            return self.get_parameter(name).value

        def _reload_if_changed(self):
            """CSV 수정시각 감시 -> step4_main.py 재실행 결과 자동 반영."""
            changed = False
            for key, loader in (("raceline", load_raceline),
                                ("centerline", load_xy_csv),
                                ("walls", load_xy_csv)):
                path = self._p(f"{key}_csv" if key != "raceline"
                               else "raceline_csv")
                if not path or not os.path.exists(path):
                    continue
                mt = os.path.getmtime(path)
                if self._mtimes.get(key) != mt:
                    try:
                        self._data[key] = loader(path)
                        self._mtimes[key] = mt
                        changed = True
                        self.get_logger().info(f"loaded {key}: {path}")
                    except Exception as e:
                        self.get_logger().warn(f"failed to load {key}: {e}")
            return changed

        def _tick(self):
            if self._reload_if_changed():
                self._dirty = True
            if self._dirty:
                self.pub.publish(self._build_markers())
                self._dirty = False

        # ---------------------------------------------------- 마커 빌드
        def _marker(self, ns, mid, mtype):
            m = Marker()
            m.header.frame_id = self._p("frame_id")
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        def _build_markers(self):
            arr = MarkerArray()
            # DELETEALL로 이전 마커 청소 (show_* 끈 것 반영)
            wipe = self._marker("", 0, Marker.LINE_STRIP)
            wipe.action = Marker.DELETEALL
            arr.markers.append(wipe)

            race = self._data.get("raceline")
            if race is not None and self._p("show_raceline"):
                m = self._marker("raceline", 0, Marker.LINE_STRIP)
                m.scale.x = 0.07
                m.color = ColorRGBA(r=1.0, g=0.55, b=0.0, a=1.0)  # 주황
                m.points = [Point(x=float(x), y=float(y), z=0.02)
                            for x, y in race["xy"]] + \
                           [Point(x=float(race["xy"][0, 0]),
                                  y=float(race["xy"][0, 1]), z=0.02)]
                arr.markers.append(m)

            if race is not None and self._p("show_speed"):
                bars = build_speed_bars(race["xy"], race["vx"],
                                        self._p("speed_scale"),
                                        int(self._p("speed_stride")))
                m = self._marker("speed_bars", 0, Marker.LINE_LIST)
                m.scale.x = 0.03
                for lo, hi, (r, g, b) in bars:
                    m.points += [Point(x=lo[0], y=lo[1], z=lo[2]),
                                 Point(x=hi[0], y=hi[1], z=hi[2])]
                    c = ColorRGBA(r=r, g=g, b=b, a=0.9)
                    m.colors += [c, c]
                arr.markers.append(m)
                rib = self._marker("speed_ribbon", 0, Marker.LINE_STRIP)
                rib.scale.x = 0.04
                rib.color = ColorRGBA(r=1.0, g=0.9, b=0.1, a=0.9)  # 노랑
                rib.points = [Point(x=p[0], y=p[1], z=p[2]) for p in
                              build_speed_ribbon(race["xy"], race["vx"],
                                                 self._p("speed_scale"))]
                arr.markers.append(rib)

            walls = self._data.get("walls")
            if walls is not None and self._p("show_walls"):
                for wid in (0, 1):   # 0=outer, 1=inner
                    pts = walls[walls[:, 2] == wid][:, :2] \
                        if walls.shape[1] >= 3 else walls[:, :2]
                    if len(pts) < 2:
                        continue
                    m = self._marker("walls", wid, Marker.TRIANGLE_LIST)
                    m.scale.x = m.scale.y = m.scale.z = 1.0
                    m.color = ColorRGBA(r=0.35, g=0.35, b=0.4,
                                        a=float(self._p("wall_alpha")))
                    m.points = [Point(x=p[0], y=p[1], z=p[2]) for p in
                                build_wall_triangles(pts,
                                                     self._p("wall_height"))]
                    arr.markers.append(m)

            center = self._data.get("centerline")
            if center is not None and self._p("show_centerline"):
                m = self._marker("centerline", 0, Marker.LINE_STRIP)
                m.scale.x = 0.03
                m.color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.6)
                m.points = [Point(x=float(x), y=float(y), z=0.01)
                            for x, y in center[:, :2]]
                arr.markers.append(m)
            return arr

    rclpy.init()
    node = RacelineVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

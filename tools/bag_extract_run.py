#!/usr/bin/env python3
"""실차 rosbag에서 '그날 실제로 돌던' 맵과 레이스라인을 그대로 꺼낸다.

왜 필요한가
-----------
맵과 레이스라인은 차 위에서 생성되고, 커밋되지 않은 채로 주행하는 일이 잦다.
rosbag_0805의 세 런이 그랬다 — bag 안의 /map(345x128, origin -8.101)과 일치하는
맵이 이 저장소에 하나도 없었고, 커밋된 my_track0805(336x129, origin -9.297)로는
x가 ~1.5 m 어긋난 다른 트랙이 된다.

그런데 bag에는 그 둘이 통째로 들어 있다:
  /map              nav_msgs/OccupancyGrid   (latched, 1회)
  /global_waypoints WpntArray                (2 Hz, 전 필드)
WpntArray는 build_map.py CSV의 9개 열을 전부 담고 있어 CSV를 무손실 복원할 수 있다.
그래서 "차에서 파일을 다시 받아오기"를 기다리지 않고 sim을 실차와 같은 트랙 위에
세울 수 있다.

출력 이름 규칙
--------------
sim_race.launch.py는 맵 이름 하나로 파일 3종을 찾는다. 그래서 launch가 그대로
집어갈 수 있는 이름으로 쓴다:

    <ws>/maps/<name>.{yaml,pgm}                      # 워크스페이스 공용 맵 폴더
    planning_pkg/config/global_trajectory/<name>_iqp_<suffix>.csv
    planning_pkg/config/centerline/<name>_centerline.csv

맵은 패키지에 설치되지 않으므로 맵만 놓고 보면 재빌드가 필요 없다. 다만
raceline/centerline CSV는 패키지에 설치되므로 colcon build 후에 반영된다.

suffix는 `max<v>ms_margin<m>m_mu<mu>` 형식이어야 한다 — tuner:=true가 이 문자열에서
mu를 파싱하기 때문이다(sim_race.launch.py:_CSV_PARAM_RE).

⚠️ bag에는 margin/mu가 안 남는다. WpntArray는 결과(기하+속도)만 담지 생성
파라미터를 담지 않는다. 그래서 --suffix 기본값은 같은 날 커밋된 my_track0805
생성 매니페스트의 값(max6.00ms/margin0.08m/mu0.65)을 물려받은 것이고, 이는
'추정'이다. 기하와 속도는 bag 그대로라 주행 재현에는 영향이 없고, tuner의
mu 표시에만 쓰인다.

사용법
------
    python3 tools/bag_extract_run.py rosbag_0805/run_0805_225904_0.db3 \
        --name my_track0805_asrun
"""
import argparse
import sqlite3
import sys
from pathlib import Path

import numpy as np

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# build_map.py가 쓰는 열 순서. planning의 CSV 로더가 이 순서를 기대한다.
CSV_HEADER = ('s_m; x_m; y_m; d_right; d_left; psi_rad; kappa_radpm; '
              'vx_mps; ax_mps2')


def read_first(db3: Path, topic: str):
    con = sqlite3.connect(str(db3))
    row = con.execute('SELECT id, type FROM topics WHERE name=?',
                      (topic,)).fetchone()
    if row is None:
        raise SystemExit(f'{db3.name}에 {topic} 이 없다')
    tid, tname = row
    data = con.execute(
        'SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 1',
        (tid,)).fetchone()
    if data is None:
        raise SystemExit(f'{topic} 메시지가 0개다')
    return deserialize_message(bytes(data[0]), get_message(tname))


def write_map(grid, name: str, out_dir: Path) -> Path:
    """OccupancyGrid -> map_server가 읽는 .pgm + .yaml."""
    info = grid.info
    a = np.asarray(grid.data, dtype=np.int8).reshape(info.height, info.width)
    # map_server 규약(negate=0, trinary): 점유=0(검정), 자유=254(흰색), 미지=205.
    img = np.full(a.shape, 205, dtype=np.uint8)
    img[(a >= 0) & (a < 50)] = 254
    img[a >= 50] = 0
    # OccupancyGrid는 원점이 좌하단이고 PGM은 좌상단부터 쓴다 -> 세로 뒤집기.
    img = np.flipud(img)

    pgm = out_dir / f'{name}.pgm'
    with open(pgm, 'wb') as f:
        f.write(b'P5\n')
        f.write(f'# extracted from rosbag /map ({name})\n'.encode())
        f.write(f'{info.width} {info.height}\n255\n'.encode())
        f.write(img.tobytes())

    q = info.origin.orientation
    yaw = float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
    (out_dir / f'{name}.yaml').write_text(
        f'image: {name}.pgm\n'
        f'resolution: {info.resolution:.6f}\n'
        f'origin: [{info.origin.position.x:.6f}, '
        f'{info.origin.position.y:.6f}, {yaw:.6f}]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.196\n'
    )
    return pgm


def write_traj(wpnts, stem: str, out_dir: Path, src: str) -> Path:
    rows = np.array([[w.s_m, w.x_m, w.y_m, w.d_right, w.d_left,
                      w.psi_rad, w.kappa_radpm, w.vx_mps, w.ax_mps2]
                     for w in wpnts], dtype=float)
    csv = out_dir / f'{stem}.csv'
    with open(csv, 'w') as f:
        f.write(f'# extracted from rosbag /global_waypoints ({src})\n')
        f.write('# 차가 실제로 추종한 레이스라인 (global_trajectory_node가 '
                'velocity_scale을 적용한 뒤의 발행값).\n')
        f.write(f'# {CSV_HEADER}\n')
        for r in rows:
            f.write('; '.join(f'{v:.7f}' for v in r) + '\n')
    return csv


def write_centerline(wpnts, name: str, out_dir: Path, src: str) -> Path:
    """레이스라인 + 좌우 여유폭에서 코리도 중앙선을 유도한다.

    bag에는 중앙선이 없다. 하지만 WpntArray의 d_left/d_right가 그 점에서 좌우
    경계까지의 거리이므로, 왼쪽 법선으로 (d_left - d_right)/2 만큼 밀면 코리도
    중앙이 된다. 시각화 전용이라 이 근사로 충분하다 (planner_stack은 중앙선이
    없어도 뜨고, 없으면 시각화만 빈다).
    """
    p = np.array([[w.x_m, w.y_m] for w in wpnts])
    psi = np.array([w.psi_rad for w in wpnts])
    off = (np.array([w.d_left for w in wpnts])
           - np.array([w.d_right for w in wpnts])) / 2.0
    # psi는 진행방향. 왼쪽 법선 = (-sin, cos).
    c = p + np.column_stack((-np.sin(psi), np.cos(psi))) * off[:, None]
    csv = out_dir / f'{name}_centerline.csv'
    with open(csv, 'w') as f:
        f.write(f'# derived from rosbag /global_waypoints ({src}): '
                'raceline + 왼쪽법선 * (d_left-d_right)/2\n')
        f.write('# x_m; y_m\n')
        for x, y in c:
            f.write(f'{x:.7f}; {y:.7f}\n')
    return csv


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', type=Path, help='.db3 파일')
    ap.add_argument('--name', required=True, help='맵/레이스라인 이름 (확장자 제외)')
    ap.add_argument('--suffix', default='max6.00ms_margin0.08m_mu0.65',
                    help='raceline suffix. bag에 안 남는 생성 파라미터라 '
                         '기본값은 같은 날 매니페스트에서 물려받은 추정치다.')
    ap.add_argument('--pkg', type=Path,
                    default=Path('planning_ws/src/planning_pkg'),
                    help='planning_pkg 소스 경로 (raceline/centerline CSV 출력용)')
    ap.add_argument('--maps-dir', type=Path, default=Path('maps'),
                    help='맵 출력 폴더. 기본 = 워크스페이스 공용 <ws>/maps')
    a = ap.parse_args()

    maps_dir = a.maps_dir
    traj_dir = a.pkg / 'config' / 'global_trajectory'
    center_dir = a.pkg / 'config' / 'centerline'
    for d in (maps_dir, traj_dir, center_dir):
        d.mkdir(parents=True, exist_ok=True)

    grid = read_first(a.bag, '/map')
    pgm = write_map(grid, a.name, maps_dir)
    print(f'map  -> {pgm}  ({grid.info.width}x{grid.info.height} '
          f'res={grid.info.resolution:.3f} '
          f'origin=({grid.info.origin.position.x:.3f}, '
          f'{grid.info.origin.position.y:.3f}))')

    gw = read_first(a.bag, '/global_waypoints')
    csv = write_traj(gw.wpnts, f'{a.name}_iqp_{a.suffix}', traj_dir, a.bag.name)
    v = np.array([w.vx_mps for w in gw.wpnts])
    s = np.array([w.s_m for w in gw.wpnts])
    print(f'traj -> {csv}  ({len(gw.wpnts)} pts  s_max={s.max():.2f} m  '
          f'vx=[{v.min():.2f}, {v.max():.2f}] m/s)')

    cl = write_centerline(gw.wpnts, a.name, center_dir, a.bag.name)
    print(f'ctr  -> {cl}  (유도값 — 시각화 전용)')
    print(f'\n실행:  ros2 launch integration_pkg stack.launch.py '
          f'env:=sim_real_car map:={a.name}')
    return 0


if __name__ == '__main__':
    sys.exit(main())

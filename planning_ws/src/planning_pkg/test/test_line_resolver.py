"""웹 레이스라인 튜너 물리·CSV 계약 (`scripts/tuner/line_resolver.py`).

이 도구의 요구사항은 "사람의 판단을 깎지 말라"이고, 그래서 물리 위반으로 저장을
막지 않는다. 대신 **깎지 않으면서도 반드시 지켜야 하는 것**들이 있고 그게 여기서
검사하는 계약이다:

  1. 원본을 절대 건드리지 않는다.
  2. 아무것도 편집하지 않았으면 출력이 원본과 같다 (무연산이 무연산이어야 한다).
  3. 사람이 준 값은 정확히 그 값으로 들어간다 (앵커 = 등호 제약, 속도 = 그대로).
  4. 사람이 무슨 짓을 해도 CSV 는 런타임 **형식** 검사를 통과한다.
     advisory 모드도 형식은 fail-closed 이므로, 이게 깨지면 튜닝 라인으로는
     차가 아예 뜨지 않는다.
  5. 벽 측정이 `build_map` 과 같은 답을 낸다.
"""

import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = PACKAGE_ROOT / 'scripts' / 'tuner' / 'line_resolver.py'
MAP_NAME = 'my_track0807'
BASE_CSV = 'my_track0807_iqp_max7.00ms_margin0.08m_mu0.68.csv'
BASE_MARGIN = 0.08


def _module():
    spec = importlib.util.spec_from_file_location(
        'planning_pkg_line_resolver_test', MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def resolver():
    return _module()


@pytest.fixture(scope='module')
def fixture_paths(resolver):
    csv_path = resolver.CSV_DIR / BASE_CSV
    if not csv_path.is_file():
        pytest.skip(f'기준 raceline 이 없습니다: {csv_path}')
    try:
        map_yaml = resolver.find_map_yaml(MAP_NAME)
    except FileNotFoundError as exc:
        pytest.skip(str(exc))
    return csv_path, map_yaml


@pytest.fixture(scope='module')
def loaded(resolver, fixture_paths):
    csv_path, map_yaml = fixture_paths
    track = resolver.TrackMap(map_yaml)
    base, meta = resolver.load_csv(csv_path)
    return track, base, meta


def _state(resolver, loaded, **kwargs):
    track, base, _ = loaded
    kwargs.setdefault('corridor_margin', BASE_MARGIN)
    kwargs.setdefault('v_max', 7.0)
    return resolver.RacelineState(base, track, **kwargs)


# --------------------------------------------------------------- 생성기 상수
def test_generator_constants_come_from_build_map_not_a_copy(resolver):
    """상수를 베껴 적으면 드리프트가 시작된다. AST 로 읽는지 확인한다."""
    constants = resolver.generator_constants()
    assert set(constants) >= {
        'CAR_HALF_WIDTH', 'CAR_HALF_LENGTH', 'MU', 'MU_PROFILE', 'DS_OUT'}
    # 사본이 아니라 실제로 build_map 소스를 읽었는지: 값이 그 파일에 있어야 한다.
    source = resolver.GENERATOR_PATH.read_text(encoding='utf-8')
    assert 'CAR_HALF_WIDTH' in source and 'MU_PROFILE' in source
    assert constants['MU'] > constants['MU_PROFILE'], (
        '물리 mu 가 계획 mu 보다 커야 한다 (게이트가 계획보다 관대해야 함)')
    assert 0.0 < constants['CAR_HALF_WIDTH'] < 0.5
    assert 0.0 < constants['DS_OUT'] <= 0.5


def test_missing_generator_constant_fails_loudly(resolver, tmp_path):
    stub = tmp_path / 'build_map.py'
    stub.write_text('MU = 0.75\n', encoding='utf-8')
    with pytest.raises(ValueError, match='상수를 못 읽음'):
        resolver.generator_constants(stub)


# --------------------------------------------------------------- 벽 측정 일치
def test_wall_measurement_matches_the_generator(resolver, loaded):
    """튜너와 build_map 이 벽을 똑같이 재야 한다.

    예전 RViz 튜너는 map_server 가 발행한 OccupancyGrid 에 `0<=v<65` 를 썼고
    생성기는 yaml 의 free_thresh 를 썼다. 두 기준이 어긋나면 한쪽이 통과시킨
    라인을 다른 쪽이 거부한다.
    """
    track, base, _ = loaded
    state = _state(resolver, loaded)
    measured = np.minimum(np.maximum(state.a_hi, 0.0),
                          np.maximum(-state.a_lo, 0.0))
    columns = resolver.as_dict(base)
    from_csv = np.minimum(columns['d_right'], columns['d_left']) + BASE_MARGIN
    assert np.abs(measured - from_csv).max() < 0.5 * track.res, (
        'build_map 이 CSV 에 적어 둔 벽 여유와 다르게 측정됩니다')


# --------------------------------------------------------------- 무연산
def test_untouched_output_reproduces_the_original(resolver, loaded):
    """편집하지 않은 저장은 기하에 대해 완전한 무연산이어야 한다.

    예전 튜너는 저장할 때마다 유한차분으로 psi/kappa 를 다시 만들어, 손도 대지
    않은 CSV 조차 kappa 가 1% 흔들리고 합성 가속도가 계획 한계를 넘었다.
    """
    _, base, _ = loaded
    state = _state(resolver, loaded)
    columns = state.output()['columns']
    assert columns.shape == base.shape
    for index, name in enumerate(resolver.CSV_COLUMNS):
        tolerance = 1e-4 if name == 'ax_mps2' else 1e-9
        assert np.abs(columns[:, index] - base[:, index]).max() < tolerance, name


def test_untouched_report_has_no_flags(resolver, loaded):
    """거짓 경고는 진짜 경고를 못 믿게 만든다."""
    state = _state(resolver, loaded)
    assert state.report()['unsafe_flags'] == []


def test_reset_returns_to_the_generator_line(resolver, loaded):
    _, base, _ = loaded
    state = _state(resolver, loaded)
    state.set_anchor(140, -0.2)
    state.solve()
    state.set_speed(60, 6.5)
    state.reset_all()
    assert not state.anchors
    assert np.abs(state.alpha).max() == 0.0
    assert np.abs(state.output()['columns'][:, 7] - base[:, 7]).max() == 0.0


# --------------------------------------------------------------- 사람 값 보존
def test_anchor_is_an_equality_constraint_not_a_clamp(resolver, loaded):
    """코리도 안이든 밖이든 사람이 준 오프셋이 정확히 그 값으로 들어간다."""
    state = _state(resolver, loaded)
    for index, alpha in ((140, -0.18), (40, 0.12), (200, -0.05)):
        state.set_anchor(index, alpha)
    state.solve()
    for index, alpha in ((140, -0.18), (40, 0.12), (200, -0.05)):
        assert state.alpha[index] == pytest.approx(alpha, abs=1e-12)


def test_anchor_outside_the_corridor_is_still_honoured(resolver, loaded):
    """벽 밖으로 끌어도 깎지 않는다 — 대신 report 가 위반으로 말해 준다."""
    state = _state(resolver, loaded)
    index = 140
    outrageous = float(state.a_lo[index]) - 0.10   # 오른쪽 벽을 10 cm 관통
    state.set_anchor(index, outrageous)
    state.solve()
    assert state.alpha[index] == pytest.approx(outrageous, abs=1e-12)
    assert 'wall' in state.report()['unsafe_flags']


def test_anchor_influence_decays_and_keeps_the_rest_of_the_line(
        resolver, loaded):
    """점 하나를 끌었을 때 트랙 전체가 다시 그려지면 안 된다.

    절대 최소곡률(curvature_bias=1)로 풀면 실제로 그렇게 된다 — 랩이 2 m 짧아지고
    앵커가 아닌 구간이 앵커보다 더 많이 움직인다. 기본값은 곡률 '변화'를
    최소화하므로 원본 모양이 남는다.
    """
    _, base, _ = loaded
    state = _state(resolver, loaded, influence_m=3.0)
    state.set_anchor(140, -0.20)
    state.solve()
    alpha = np.abs(state.alpha)
    assert alpha.max() == pytest.approx(0.20, abs=0.05), (
        '앵커가 아닌 구간이 앵커보다 크게 움직였습니다')
    far = alpha[(140 + int(round(12.0 / state.ds0))) % state.n]
    assert far < 0.05, '12 m 떨어진 곳까지 끌려왔습니다'
    length = state.output()['s'][-1]
    assert abs(length - base[-1, 0]) < 0.5, '랩 길이가 크게 바뀌었습니다'


def test_speed_is_taken_literally_and_pins_are_not_overwritten(
        resolver, loaded):
    state = _state(resolver, loaded)
    state.set_speed(60, 6.40)
    state.set_speed(200, 4.20)
    assert state.v[60] == pytest.approx(6.40)
    assert state.v[200] == pytest.approx(4.20)
    # 바로 옆을 다시 편집해도 이미 사람이 정한 값은 그대로 남는다.
    state.set_speed(62, 5.10)
    assert state.v[60] == pytest.approx(6.40)
    assert state.v[62] == pytest.approx(5.10)
    assert state.pinned[60] and state.pinned[62]


def test_speed_edits_are_not_projected_onto_physics(resolver, loaded):
    """마찰 한계를 넘겨도 되돌리지 않는다. 그게 이 도구의 요구사항이다."""
    state = _state(resolver, loaded)
    corner = int(np.argmax(np.abs(state.base[:, 6])))
    state.set_speed(corner, 7.0)
    assert state.v[corner] == pytest.approx(7.0)
    report = state.report()
    assert 'friction' in report['unsafe_flags']
    assert report['friction']['max_required_mu'] > report[
        'friction']['physical_mu']


# --------------------------------------------------------------- 런타임 형식
def _assert_runtime_format_contract(columns):
    """global_trajectory_node.ValidateTrajectory 의 **형식** 검사를 그대로 재현.

    advisory 모드는 물리만 완화하고 형식은 그대로 거부하므로, 이걸 못 지키는
    CSV 로는 차가 아예 뜨지 않는다. 사람이 무슨 편집을 해도 지켜져야 한다.
    """
    assert len(columns) >= 8
    assert np.isfinite(columns).all()
    assert (columns[:, 0] >= 0.0).all()
    assert (columns[:, 3] >= 0.0).all() and (columns[:, 4] >= 0.0).all()
    assert (columns[:, 7] >= 0.0).all()
    spacing = np.diff(columns[:, 0])
    assert (spacing > 1e-5).all(), 's 가 단조 증가하지 않음'
    geometric = np.hypot(np.diff(columns[:, 1]), np.diff(columns[:, 2]))
    assert (geometric > 1e-5).all()
    assert (np.abs(geometric - spacing) <=
            np.maximum(0.02, 0.20 * spacing)).all(), 's/기하 간격 불일치'
    numeric_ax = (
        columns[1:, 7] ** 2 - columns[:-1, 7] ** 2) / (2.0 * spacing)
    assert np.abs(numeric_ax - columns[:-1, 8]).max() <= 1.0, 'ax↔v 불일치'
    assert spacing.max() <= spacing.min() * 1.25, '간격 균일성 위반'
    closure = math.hypot(
        columns[0, 1] - columns[-1, 1], columns[0, 2] - columns[-1, 2])
    assert closure <= spacing.max() * 2.5, '폐루프가 닫히지 않음'


def test_untouched_output_satisfies_the_runtime_format_contract(
        resolver, loaded):
    _assert_runtime_format_contract(_state(resolver, loaded).output()['columns'])


def test_extreme_human_edits_still_satisfy_the_format_contract(
        resolver, loaded):
    """말이 안 되는 편집도 '형식이 깨진 CSV'로는 이어지지 않아야 한다."""
    state = _state(resolver, loaded)
    # 좌우로 번갈아 크게 끌고, 코너에서 속도를 최대로, 직선에서 최소로.
    for offset, index in enumerate(range(0, state.n, 37)):
        limit = state.a_hi[index] if offset % 2 else state.a_lo[index]
        state.set_anchor(index, float(limit) * 0.9)
    state.solve()
    kappa = np.abs(state.base[:, 6])
    for index in np.argsort(kappa)[-5:]:
        state.set_speed(int(index), state.v_max)
    for index in np.argsort(kappa)[:5]:
        state.set_speed(int(index), state.v_min)
    columns = state.output()['columns']
    _assert_runtime_format_contract(columns)
    # 그리고 그 편집은 실제로 반영돼 있어야 한다 (형식을 지키려고 깎으면 안 됨).
    assert state.report()['unsafe_flags'], '위반이 보고되지 않았습니다'


# --------------------------------------------------------------- 저장/이어하기
def test_tuned_name_does_not_collide_with_the_original_glob(resolver):
    import fnmatch
    tuned = resolver.tuned_csv_name(BASE_CSV)
    assert tuned == 'my_track0807_tuned_iqp_max7.00ms_margin0.08m_mu0.68.csv'
    assert not fnmatch.fnmatch(tuned, 'my_track0807_iqp_*.csv'), (
        '원본 glob 이 튜닝본을 집으면 launch 가 "여러 CSV 매칭"으로 죽는다')
    assert fnmatch.fnmatch(tuned, 'my_track0807_tuned_iqp_*.csv')
    assert resolver.tuned_csv_name(tuned) == tuned, '두 번 적용해도 같아야 함'


def test_save_never_touches_the_original_and_resumes_exactly(
        resolver, loaded, tmp_path):
    _, base, meta = loaded
    original = (resolver.CSV_DIR / BASE_CSV).read_bytes()
    state = _state(resolver, loaded)
    state.set_anchor(140, -0.18)
    state.set_anchor(40, 0.12)
    state.solve()
    state.set_speed(60, 6.40)
    target = tmp_path / resolver.tuned_csv_name(BASE_CSV)
    result = state.save(target, base_name=BASE_CSV, base_meta=meta)

    assert (resolver.CSV_DIR / BASE_CSV).read_bytes() == original, (
        '원본이 수정됐습니다')
    header = target.read_text(encoding='utf-8')
    assert f"base_generation_id={meta.get('generation_id', '')}" in header
    assert 'unsafe_flags=' in header

    payload = json.loads(Path(result['state']).read_text(encoding='utf-8'))
    resumed = _state(resolver, loaded)
    ok, why = resumed.apply_state(payload)
    assert ok, why
    assert resumed.anchors == state.anchors
    assert np.array_equal(resumed.pinned, state.pinned)
    assert np.abs(resumed.alpha - state.alpha).max() == 0.0
    assert np.abs(
        resumed.output()['columns'] - state.output()['columns']).max() == 0.0


def test_resume_is_refused_when_the_generator_regenerated_the_line(
        resolver, loaded, tmp_path):
    """mkcsv 를 다시 돌려 웨이포인트 수가 바뀌면 옛 앵커 인덱스는 무의미하다."""
    state = _state(resolver, loaded)
    state.set_anchor(140, -0.1)
    state.solve()
    target = tmp_path / resolver.tuned_csv_name(BASE_CSV)
    result = state.save(target, base_name=BASE_CSV)
    payload = json.loads(Path(result['state']).read_text(encoding='utf-8'))
    payload['n'] = payload['n'] + 7
    ok, why = _state(resolver, loaded).apply_state(payload)
    assert not ok
    assert '웨이포인트 수' in why


def test_format_violation_is_reported_separately_from_physics(
        resolver, loaded):
    """형식 위반은 "차가 안 뜬다"는 뜻이라 물리 경고와 구분돼야 한다.

    저장을 막지 않는 도구이므로, 이게 화면에 안 보이면 사람은 튜닝한 라인으로
    트랙에 나갔다가 노드가 안 뜨는 이유를 모른 채 시간을 버린다.
    """
    state = _state(resolver, loaded)
    report = state.report()
    assert report['format']['loadable'], report['format']['problems']
    assert 'format' not in report['unsafe_flags']
    assert report['format']['spacing_uniformity'] < 1.25

    # 속도를 코너에서 최대로 올려 물리는 깨고 형식은 지키는 편집.
    corner = int(np.argmax(np.abs(state.base[:, 6])))
    state.set_speed(corner, state.v_max)
    physics_only = state.report()
    assert 'friction' in physics_only['unsafe_flags']
    assert 'format' not in physics_only['unsafe_flags']
    assert physics_only['format']['loadable']


def test_chord_uniform_resampling_keeps_the_line_loadable(resolver, loaded):
    """호장 균일화만으로는 부족하다 — 런타임이 보는 건 현(弦) 간격이다."""
    state = _state(resolver, loaded)
    for order, index in enumerate(range(0, state.n, 37)):
        limit = state.a_hi[index] if order % 2 else state.a_lo[index]
        state.set_anchor(index, float(limit) * 0.9)
    state.solve()
    without = resolver.respline_uniform(state.xy, 0.1, iterations=0)
    spacing = np.hypot(np.diff(without['xy'][:, 0]),
                       np.diff(without['xy'][:, 1]))
    assert spacing.max() > spacing.min() * 1.25, (
        '이 편집이 더는 균일성을 깨지 않습니다 — 회귀 테스트를 다시 만드세요')
    assert state.report()['format']['loadable'], '균일화가 문제를 못 고쳤습니다'

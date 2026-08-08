"""원본 라인 / 사람 튜닝 라인(_tuned) 분리 계약.

레이스라인 튜너는 `build_map` 원본을 절대 덮어쓰지 않고 `<맵>_tuned_iqp_*.csv` 로
따로 저장한다. 그 라인은 사람의 직감을 그대로 담으므로 마찰원·조향 한계를 넘을 수
있고, 그걸로 주행하려면 런타임 물리 검사를 advisory 로 내려야 한다.

여기서 지키는 것은 그 완화가 **_tuned 경로에만** 적용된다는 것이다. 원본 라인이
조용히 advisory 로 떨어지면 검증되지 않은 CSV 로 실차가 달리게 된다.
"""

import ast
import re
from pathlib import Path
from types import SimpleNamespace


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[2]
PLANNER_STACK_LAUNCH = PACKAGE_ROOT / 'launch' / 'planner_stack.launch.py'
REAL_CAR_LAUNCH = (
    WORKSPACE_ROOT / 'integration_ws' / 'src' / 'integration_pkg' /
    'launch' / 'real_car.launch.py')
NODE_SOURCE = PACKAGE_ROOT / 'src' / 'global_trajectory_node.cpp'
PARAMS_YAML = PACKAGE_ROOT / 'config' / 'params.yaml'


def _load(path, name=None):
    """launch 파일에서 LINE_VARIANTS + split_line_variant 만 AST로 떼어 실행한다.

    launch 파일 전체를 import 하면 launch/launch_ros/ament_index_python 이 필요해
    ROS 를 source 하지 않은 셸에서 이 테스트가 import 오류로 죽는다. 검사 대상은
    순수 문자열 처리뿐이므로 그 두 정의만 꺼내 쓴다 (build_map 안전 함수를
    test_tuner_safety_model.py 가 꺼내 쓰는 것과 같은 방식).
    """
    del name
    tree = ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    selected = []
    found = set()
    for statement in tree.body:
        if isinstance(statement, ast.Assign) and \
                len(statement.targets) == 1 and \
                isinstance(statement.targets[0], ast.Name) and \
                statement.targets[0].id == 'LINE_VARIANTS':
            selected.append(statement)
            found.add('LINE_VARIANTS')
        elif getattr(statement, 'name', None) == 'split_line_variant':
            selected.append(statement)
            found.add('split_line_variant')
    missing = {'LINE_VARIANTS', 'split_line_variant'} - found
    assert not missing, f'{path.name} 에 {sorted(missing)} 가 없습니다'
    module = ast.Module(body=selected, type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {}
    exec(compile(module, str(path), 'exec'), namespace)  # noqa: S102
    return SimpleNamespace(**{
        key: value for key, value in namespace.items()
        if not key.startswith('__')})


def test_both_launch_copies_split_identically():
    """두 launch 는 서로 다른 워크스페이스라 import 할 수 없어 사본으로 둔다.

    사본이 어긋나면 PF 는 <맵>_tuned.yaml 을 찾다가 죽거나, 반대로 레이스라인이
    원본으로 조용히 되돌아간다. 둘 다 주행 직전에야 드러난다.
    """
    planner = _load(PLANNER_STACK_LAUNCH, 'planner_stack_launch_probe')
    real_car = _load(REAL_CAR_LAUNCH, 'real_car_launch_probe')
    assert planner.LINE_VARIANTS == real_car.LINE_VARIANTS
    for name in ('my_track0808', 'my_track0808_tuned', 'my_track0806_3',
                 'tuned', 'my_tuned_track', '', 'a_tuned'):
        assert planner.split_line_variant(name) == \
            real_car.split_line_variant(name), name


def test_split_line_variant_only_strips_a_real_suffix():
    planner = _load(PLANNER_STACK_LAUNCH, 'planner_stack_launch_probe2')
    split = planner.split_line_variant
    assert split('my_track0808') == ('my_track0808', '')
    assert split('my_track0808_tuned') == ('my_track0808', 'tuned')
    # 접미사만으로 이루어진 이름은 맵 이름이 비어 버리므로 쪼개지 않는다.
    assert split('tuned') == ('tuned', '')
    # 중간에 들어 있는 'tuned' 는 변형이 아니다.
    assert split('my_tuned_track') == ('my_tuned_track', '')


def test_advisory_is_only_reachable_through_a_line_variant():
    """planner_stack 에서 validation_mode 를 쓰는 곳은 라인 변형 분기 하나뿐."""
    source = PLANNER_STACK_LAUNCH.read_text(encoding='utf-8')
    assert source.count("'global_trajectory/validation_mode'") == 1
    assert source.count("'advisory'") == 1
    tree = ast.parse(source, filename=str(PLANNER_STACK_LAUNCH))
    # 그 대입이 `if line_variant:` 안에 있는지 확인한다.
    guarded = False
    for node in ast.walk(tree):
        if not isinstance(node, ast.If):
            continue
        if not (isinstance(node.test, ast.Name) and
                node.test.id == 'line_variant'):
            continue
        if 'validation_mode' in ast.dump(node):
            guarded = True
    assert guarded, 'validation_mode=advisory 가 line_variant 분기 밖에 있습니다'


def test_default_validation_mode_is_strict_everywhere():
    """노드 기본값과 params.yaml 이 모두 strict 여야 한다."""
    node_source = NODE_SOURCE.read_text(encoding='utf-8')
    assert re.search(
        r'declare_parameter\(\s*"global_trajectory/validation_mode"\s*,\s*"strict"\s*\)',
        node_source), '노드 기본값이 strict 가 아닙니다'
    params = PARAMS_YAML.read_text(encoding='utf-8')
    assert re.search(
        r'^\s*global_trajectory/validation_mode:\s*strict\s*$',
        params, re.MULTILINE), 'params.yaml 기본값이 strict 가 아닙니다'


def test_unknown_validation_mode_is_rejected_not_defaulted():
    """오타가 조용히 strict/advisory 로 떨어지면 안 된다.

    advisory 로 떨어지는 쪽은 검증 없이 주행하는 것이라 반드시 죽어야 하고,
    strict 로 떨어지는 쪽도 사람이 advisory 를 의도했는데 노드가 안 뜨는 이유를
    설명해 주지 않는다.
    """
    node_source = NODE_SOURCE.read_text(encoding='utf-8')
    assert 'validation_mode must be' in node_source
    assert re.search(
        r'validation_mode\s*!=\s*"strict"', node_source), \
        '알 수 없는 모드를 거부하는 분기가 없습니다'


def test_physics_checks_are_the_only_ones_relaxed():
    """advisory 는 마찰원/조향만 완화한다. 형식 검사는 두 모드 모두 fail-closed.

    형식(NaN·s 단조·간격 균일·폐루프·ax↔v)이 깨진 CSV 는 사람의 직감 문제가
    아니라 하류 컨트롤러 수학이 깨지는 문제라 어느 모드에서도 통과시키면 안 된다.
    """
    node_source = NODE_SOURCE.read_text(encoding='utf-8')
    # validation_advisory_ 가 감싸는 return false 는 정확히 두 개(조향/마찰)여야 한다.
    assert node_source.count('if (!validation_advisory_) {') == 2
    for format_error in (
            'contains invalid/non-finite data',
            'has a negative s/bound/speed',
            's is not strictly increasing at row',
            'has inconsistent s/geometry spacing',
            'ax profile mismatch at row',
            'waypoint spacing is not uniform',
            'is not a closed loop'):
        index = node_source.index(format_error)
        # 각 형식 검사 뒤 300자 안에 advisory 우회 분기가 없어야 한다.
        window = node_source[index:index + 300]
        assert 'validation_advisory_' not in window, format_error

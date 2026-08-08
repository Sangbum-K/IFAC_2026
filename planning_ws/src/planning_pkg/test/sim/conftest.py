"""이 폴더는 pytest 대상이 아니다.

`my_track_test.py` 는 이미 떠 있는 시뮬(sim_race.launch.py)에
붙어서 검사하는 실행 스크립트다. 이름이 `*_test.py` 라 pytest 가 수집하려 들고,
launch_testing 훅이 entrypoint 를 찾느라 모듈을 import 하는데, 그 시점에
CSV 조회 결과에 따라 `sys.exit()` 이 돌아 **패키지 전체 수집이 실패**한다.
(실제로 my_track 라인이 삭제된 뒤 `pytest test/` 가 통째로 죽어 있었다.)

실행은 그대로 스크립트로 한다:
    python3 test/sim/my_track_test.py [<맵>]
"""

collect_ignore_glob = ['*.py']

"""pitwall — 초경량 텔레메트리 (unicorn/HMCL pitwall-telemetry 대응).

토픽 정의 없이 코드 내부 변수를 한 줄로 노출한다:

    import pitwall
    pitwall.init(self)              # 노드 __init__에서 1회
    pitwall.log("cmd_speed", v)     # -> /pitwall/cmd_speed (std_msgs/Float32)
    pitwall.event("벽 충돌 감지")     # -> /pitwall/events   (std_msgs/String)

규칙:
  - publisher는 이름별로 lazy 생성 후 캐시.
  - init을 안 했거나 실패하면 모든 호출이 조용히 no-op
    (계측이 주행 코드를 죽이면 안 된다).
  - event 포맷: "[<sec>.<nsec>][<node_name>] <text>"

시각화:
  ros2 run rqt_plot rqt_plot /pitwall/cmd_speed/data
  ros2 topic echo /pitwall/events
  ros2 bag record -s mcap -e '/pitwall/.*'
"""

_node = None
_float_pubs = {}
_event_pub = None


def init(node):
    """rclpy Node를 등록한다. 실패해도 예외를 내지 않는다."""
    global _node, _event_pub, _float_pubs
    try:
        _node = node
        _float_pubs = {}
        from std_msgs.msg import String
        _event_pub = node.create_publisher(String, '/pitwall/events', 10)
    except Exception:
        _node = None
        _event_pub = None


def log(name, value):
    """숫자 value를 /pitwall/<name> (Float32)으로 발행한다."""
    if _node is None:
        return
    try:
        pub = _float_pubs.get(name)
        if pub is None:
            from std_msgs.msg import Float32
            pub = _node.create_publisher(Float32, '/pitwall/' + name, 10)
            _float_pubs[name] = pub
        from std_msgs.msg import Float32
        msg = Float32()
        msg.data = float(value)
        pub.publish(msg)
    except Exception:
        pass


def event(text):
    """문자열 이벤트를 /pitwall/events (String)으로 발행한다."""
    if _node is None or _event_pub is None:
        return
    try:
        from std_msgs.msg import String
        t = _node.get_clock().now().to_msg()
        msg = String()
        msg.data = f'[{t.sec}.{t.nanosec:09d}][{_node.get_name()}] {text}'
        _event_pub.publish(msg)
    except Exception:
        pass

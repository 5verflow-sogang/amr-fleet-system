"""RViz2 E-Stop 버튼 (interactive marker). 로봇 네임스페이스 안에서 하나씩 띄운다.

원 명세 7장 "비상 정지(E-Stop) 버튼을 누르면 로봇이 즉시 물리적으로 정지해야 한다"의 버튼이다.
차체(<ns>/base_footprint) 위에 상자를 띄우고, RViz2 InteractiveMarkers 디스플레이에서 클릭하면
safety/estop(std_srvs/SetBool)을 불러 토글한다. 정지 자체는 안전 노드(safety_node)가 한다.

구독: safety/status (amr_msgs/SafetyStatus. ESTOP 여부로 다음 요청과 모양을 정한다)
클라이언트: safety/estop (std_srvs/SetBool)
interactive marker 서버: estop_button (토픽 estop_button/update·feedback, 서비스 estop_button/get_interactive_markers)
판단 규칙은 estop_button.py
"""

import time
from typing import Optional

from interactive_markers import InteractiveMarkerServer
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import SetBool
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)

from amr_msgs.msg import SafetyStatus
from amr_safety.estop_button import ButtonLook, button_look, next_request, status_level

MARKER_NAME = 'estop'
BOX_SIZE = (0.35, 0.35, 0.15)    # m. 차체(0.6×0.4 m) 위에서 눈에 띄고 클릭하기 쉬운 크기
TEXT_HEIGHT = 0.12               # m
REFRESH_PERIOD_S = 0.5           # 상태 공백(안전 노드 꺼짐) 감시 주기


def build_marker(frame_id: str, height: float, look: ButtonLook) -> InteractiveMarker:
    """frame_id 원점 위 height [m]에 버튼 상자와 글자를 둔 interactive marker."""
    r, g, b = look.rgb
    box = Marker(type=Marker.CUBE)
    box.scale.x, box.scale.y, box.scale.z = BOX_SIZE
    box.color.r, box.color.g, box.color.b, box.color.a = r, g, b, 0.9

    text = Marker(type=Marker.TEXT_VIEW_FACING, text=look.text)
    text.pose.position.z = 0.5 * BOX_SIZE[2] + TEXT_HEIGHT
    text.scale.z = TEXT_HEIGHT
    text.color.r = text.color.g = text.color.b = text.color.a = 1.0

    control = InteractiveMarkerControl(
        name='button', interaction_mode=InteractiveMarkerControl.BUTTON, always_visible=True)
    control.markers = [box, text]

    marker = InteractiveMarker(name=MARKER_NAME, scale=0.5)
    marker.header.frame_id = frame_id
    marker.pose.position.z = height
    marker.pose.orientation.w = 1.0
    marker.controls = [control]
    return marker


class EStopButtonNode(Node):
    def __init__(self):
        super().__init__('estop_button')
        base_frame = self.declare_parameter('base_frame', '').value
        self.height = float(self.declare_parameter('height', 0.8).value)
        ns = self.get_namespace().strip('/')
        self.base_frame = base_frame or (f'{ns}/base_footprint' if ns else 'base_footprint')

        self.last_level: Optional[int] = None
        self.last_status_time: Optional[float] = None   # 단조 시계 [s]. 시뮬 시간이 멈춰도 공백을 잰다
        self.pending = False
        self.shown: Optional[ButtonLook] = None

        self.client = self.create_client(SetBool, 'safety/estop')
        self.create_subscription(SafetyStatus, 'safety/status', self.on_status, 10)
        self.server = InteractiveMarkerServer(self, 'estop_button')
        self.create_timer(REFRESH_PERIOD_S, self.refresh)
        self.refresh()
        self.get_logger().info(
            f'E-Stop 버튼: {self.base_frame} 위 {self.height} m, 서비스 {self.client.srv_name}')

    def level(self) -> Optional[int]:
        age = None if self.last_status_time is None else time.monotonic() - self.last_status_time
        return status_level(self.last_level, age)

    def on_status(self, msg: SafetyStatus):
        self.last_level = int(msg.level)
        self.last_status_time = time.monotonic()
        self.refresh()

    def refresh(self):
        """모양이 바뀌었을 때만 마커를 다시 보낸다."""
        look = button_look(self.level(), self.pending)
        if look == self.shown:
            return
        self.shown = look
        self.server.insert(build_marker(self.base_frame, self.height, look),
                           feedback_callback=self.on_feedback)
        self.server.applyChanges()

    def on_feedback(self, feedback: InteractiveMarkerFeedback):
        if feedback.event_type != InteractiveMarkerFeedback.BUTTON_CLICK:
            return
        if self.pending:
            self.get_logger().info('E-Stop 버튼: 이전 요청의 응답을 기다리는 중이라 클릭을 무시한다')
            return
        if not self.client.service_is_ready():
            self.get_logger().error(f'E-Stop 버튼: {self.client.srv_name} 서비스가 없다 (안전 노드 확인)')
            return
        data = next_request(self.level())
        self.get_logger().warn(f'E-Stop 버튼 클릭: {"걸기" if data else "해제"} 요청')
        self.pending = True
        self.refresh()
        self.client.call_async(SetBool.Request(data=data)).add_done_callback(self.on_response)

    def on_response(self, future):
        self.pending = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - 서비스 실패는 로그만 남기고 버튼은 계속 쓴다
            self.get_logger().error(f'E-Stop 요청 실패: {exc!r}')
        else:
            if response.success:
                self.get_logger().info(f'E-Stop 응답: {response.message}')
            else:
                self.get_logger().error(f'E-Stop 요청 거부: {response.message}')
        self.refresh()


def main(args=None):
    rclpy.init(args=args)
    node = EStopButtonNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()

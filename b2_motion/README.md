# B2 자세 서비스 연결

실기 drivers.launch.py가 이 어댑터를 시작한다. 시작 시 자세·속도 명령을 보내지 않는다.
C++ 노드이며 서비스 응답은 비동기로 전달한다. SDK 응답을 기다리는 동안 실행 스레드를 막지 않는다.
HMI의 /stand_up, /stand_down, /balance_stand, /recovery_stand, /damp
(std_srvs/Trigger)를 /b2/motion/set_mode (b2_interface/SetMotionMode)의
이름 있는 상수로 변환한다. vendor 드라이버는 변경하지 않는다.
성공은 SDK success와 result_code=0을 모두 확인한다. 로봇의 실제 자세 도달은 별도 상태 확인 대상이다.
응답 대기 중 다른 자세 요청은 거절한다. 기본 5초 timeout은 실패를 반환하되
기존 명령의 결과가 확인될 때까지 재요청을 막는다. 자동 재시도는 하지 않는다.
SDK 전송을 사용하는 실기 bringup용이며 legacy ros_bridge와 동시에 실행하지 않는다.
기존 HMI의 정지·안전·팔 권한 조건은 그대로 적용된다.

실기 전체 ROS 그래프는 Fast DDS로 맞춘다. CycloneDDS/Fast DDS 혼합 서비스
시험에서 응답 유실을 재현했으므로 서비스 발견만으로 연결 성공을 판단하지 않는다.
Unitree SDK2 내부 DDS participant는 이 ROS RMW 선택과 별개다.

계단 모드 서비스 `/b2/stairs/enter`는 `SetMotionMode.VISION_WALK_ON`으로
VisionWalk(true)를 요청하고, `/b2/stairs/exit`는 `CLASSIC_WALK_ON`으로
ClassicWalk(true)를 요청한다. 두 API 모두 SDK 성공 응답을 확인한다. gait 번호를 추측하지 않는다.
상행·하행 모두 Vision Walk를 사용하며 방향·속도는 계단 제어기 프로파일에서 구분한다.
물리적인 모드 전환 지연과 보행 안정성은 실제 B2에서 검증해야 한다.

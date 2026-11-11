# B2 SDK 카메라 → ROS Image·CameraInfo

Unitree SDK2의 `FrontVideoClient` 또는 `BackVideoClient.GetImageSample`로 받은
영상 바이트를 OpenCV로 디코딩해 `sensor_msgs/Image`와 `CameraInfo`로 발행한다.
카메라 API만 사용하며 보행·자세·운동 제어 client를 생성하지 않는다.
기존 `robot/third_party/b2_driver/b2_base/upstream/unitree_sdk2`를 빌드 시 참조하며
외부 소스는 복사하거나 수정하지 않는다.

## 실기 실행

Linux x86_64 또는 aarch64의 ROS 2 Jazzy 환경에서 실행한다. 기본 빌드는
고정된 서브모듈의 CMake target을 우선 사용한다. 소스가 없거나
`-DB2_CAMERA_USE_PINNED_SDK=OFF`를 명시했을 때만 설치된 `unitree_sdk2`를 찾는다.
`B2_CAMERA_UNITREE_SDK2_DIR`로 기존 SDK 소스 경로를 지정할 수 있다. SDK 예제는 빌드하지 않는다.

```bash
cd ~/shalom_ws
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src/shalom --packages-select b2_camera
source install/setup.bash
ros2 launch b2_camera camera.launch.py \
  network_interface:=<B2에_연결된_NIC> \
  camera_info_url:=file:///absolute/path/b2_front_camera.yaml
```

측정한 보정 YAML은 `camera_info_manager`가 로드한다. K·D·R·P, 영상 크기와
optical frame은 실제 카메라 설정에 맞아야 한다. 가상의 초점거리나 장착 TF를 만들지 않는다.
전면 파일의 `camera_name`은 `b2_front_camera`, 후면 파일은 `b2_back_camera`다.
전후면을 각각 보정해 별도 YAML로 관리하며, 이름이 선택한 `camera_side`와 다르면
SDK 연결 전에 거절한다. 파일 관리 양식과 구분표는
[캘리브레이션 정리](config/calibration/README.md)를 따른다.
디코딩한 크기가 보정 크기와 다르면 프레임을 버리며 K/P를 자동 확대·축소하지 않는다.
기본 optical frame은 B2 URDF의 전면 `f_oc_link`·후면 `r_oc_link`다.
`b2_bringup.launch.py`가 기본으로 실행하는 `robot_state_publisher`가
`base_link`에서 각 카메라까지의 고정 TF를 발행한다. 카메라 launch만 단독 실행하면
이 TF 발행기는 시작되지 않으므로 B2 description도 실행해야 한다. 같은 TF를 중복 발행하지 않는다.
`frame_id`를 바꾸는 경우 해당 optical frame까지의 TF는 호출자가 제공한다.

제공된 `corners` 전방 카메라의 1920×1080 raw 보정값은
[config/calibration](config/calibration/README.md)에 보정값과 출처를 보존했다.
해당 카메라에서는 `camera_side:=front`와
`camera_info_url:=package://b2_camera/config/calibration/b2_front_1920x1080.yaml`로
명시적으로 선택한다. 일반 launch의 빈 URL 기본값과 후방 보정 요구는 유지한다.
이 파일의 fisheye skew는 L2 보정에서 처리하며, L1은 영상 보정을 수행하지 않는다.

| 인자 | 기본값·의미 |
| --- | --- |
| `network_interface` | 필수, B2 Ethernet NIC |
| `camera_side` | `front`; 후방은 `back` |
| `dds_domain` | `0`, SDK DDS domain(0..232) |
| `publish_rate_hz` | `5.0`, RPC 완료 뒤 다음 요청까지의 polling 상한(0..60); 큰 프레임은 전송 예산에 맞춰 더 낮아짐 |
| `rpc_timeout_s` | `0.5`, SDK RPC 제한(0..10초, 양수) |
| `camera_info_url` | 측정한 `file://` 또는 `package://` 보정 URL |
| `require_calibration` | `true`; `false`는 보정 영상 수집 전용 |
| `namespace` | 빈 값이면 `b2/front_camera` 또는 `b2/back_camera` |
| `frame_id` | 빈 값이면 URDF의 `f_oc_link`(전면) 또는 `r_oc_link`(후면) |

기본 출력은 `/b2/front_camera/image_raw`, `/b2/front_camera/camera_info`이며
후방 선택 시 `/b2/back_camera/…`로 분리된다. Image는 디코딩된 `bgr8`이고
CameraInfo와 모두 `SensorDataQoS`를 사용한다. 전·후방을 별도로 실행할 때 각 카메라의
실측 YAML을 지정한다. 입력 영상은 raw이며 L2 `apriltag_perception`의 기본 보정 경로를 사용한다.

전·후면을 함께 구성할 때는 이 launch를 `camera_side:=front`와 `camera_side:=back`으로
각각 실행하고 각 카메라의 보정 YAML을 지정한다. 노드 namespace·영상 토픽·URDF frame이
분리되므로 launch 설정은 충돌하지 않는다. 실제 B2의 두 SDK 스트림 동시 수신과 속도는
실기에서 확인해야 한다. 두 관측을 사용하는 도킹 경로는
[이중 카메라 도킹](../../../l3_control/docking/docs/multi_camera.md)의 공통 기준 보정과
유효성 신호 구성을 사용한다.

SDK는 촬영 시각을 반환하지 않는다. 따라서 **RPC 수신 완료 시각**을 Image와
CameraInfo에 동일하게 기록한다. 센서의 실제 노출 시각이 아니며 네트워크·SDK 버퍼 지연이
포함될 수 있다. 정밀 시간 동기화 또는 이동 중 pose 오차의 검증을 대신하지 않는다.
실패·빈 샘플·디코딩 오류·보정 불일치에서는 발행하지 않으며 이전 영상을 재발행하지 않는다.

단일 executor와 timer에서 한 번에 RPC 하나만 실행한다. 요청 중 timer를 취소하고
완료 후 다시 시작해 밀린 요청을 누적하지 않는다. 종료 시 작업 thread를 기다리는
별도 루프는 없고 진행 중 RPC는 설정한 SDK timeout을 사용한다. 공급사 정적 라이브러리의
내부 destructor 동작과 최악 종료 시간은 공개 헤더만으로 보증할 수 없으므로 실기에서 확인한다.

ROS raw 영상은 카메라별 50 MB/s 이하로 요청 간격을 추가 제한한다. 1080p BGR의
경우 전송 예산에 따른 최소 간격은 약 124 ms이며 기본 5 Hz의 200 ms 간격은 유지한다.
이것은 ROS loopback 전송량 기준이며 B2 Ethernet의 압축 영상 처리율이나 보장 FPS가 아니다.
높은 polling 설정에서 전송 중인 프레임이 DDS history에서 반복 퇴출돼 수신이 멈추는
현상을 방지한다. 오류 응답·미발행 프레임은 이 영상 전송 예산을 소비하지 않는다.

SDK에 내장된 CycloneDDS와 같은 프로세스의 ROS RMW가 충돌하지 않도록 launch는
이 노드에만 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`와
`FASTRTPS_DEFAULT_PROFILES_FILE=<설치된 패키지>/config/fastdds.xml`을 지정한다.
이 ROS participant는 loopback `127.0.0.1`의 UDPv4만 사용하고 기본 SHM/외부 NIC
transport를 끈다. 따라서 Ethernet NIC가 함께 있어도 프로젝트의 loopback CycloneDDS
ROS graph로 Image·CameraInfo를 전달한다. 다른 노드의 RMW·DDS 설정은 변경하지 않는다.
XML 외의 기본 transport를 추가하는 Jazzy의 `LOCALHOST` 처리를 피하도록 이 자식
노드에만 `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`을 지정한다. 실제 ROS 통신 범위는
XML whitelist의 loopback으로 제한되며 외부 NIC를 사용하도록 넓히지 않는다.
SDK의 B2 영상 연결은 별도의 `network_interface`를 사용한다.
카메라 노드에만 `RMW_FASTRTPS_USE_QOS_FROM_XML=1`을 지정해 비동기 writer와
64 KiB/1 ms flow controller를 사용한다. 6.2 MB 프레임을 한 번에 UDP로 몰아 보내지
않도록 분산하며, realloc 가능한 history와 data sharing 비활성화를 명시한다.
작은 OS UDP 버퍼에서도 전후방 수신을 확인했으며 시스템 sysctl은 변경하지 않았다.
설정 근거는 [Fast DDS 대용량 전송](https://fast-dds.docs.eprosima.com/en/v2.14.5/fastdds/use_cases/large_data/large_data.html)과
[ROS Jazzy RMW의 XML 적용 규칙](https://github.com/ros2/rmw_fastrtps/blob/jazzy/README.md#full-qos-configuration)이다.
SDK 공유 DDS runtime 바이너리와 버전 링크는 기존 파일 그대로
패키지 설치 prefix 아래 `lib/b2_camera/sdk`에 설치한다. 전역 ROS library 경로에 섞지 않으며 실행파일의
상대 RPATH가 SDK의 전이 의존성까지 찾으므로 설치 후 소스 checkout 경로에 의존하지 않는다.
고정 SDK를 사용할 때 SDK와 포함 라이브러리의 license 파일도 패키지 share에 함께 설치한다.

## 보정용 영상 수집

실제 보정 작업은 [b2_camera_calibration](../b2_camera_calibration/README.md)의
전용 launch로 카메라·GUI·저장 경로를 함께 구성한다. 아래는 SDK 영상 수집만 별도로
실행할 때의 명령이다.

아직 YAML이 없는 경우에만 명시적으로 다음 모드로 실행한다.

```bash
ros2 launch b2_camera camera.launch.py \
  network_interface:="$B2_NIC" camera_side:=front require_calibration:=false

ros2 launch b2_camera camera.launch.py \
  network_interface:="$B2_NIC" camera_side:=back require_calibration:=false
```

URL 없이 실행하면 기존 `~/.ros/camera_info` 파일을 자동 로드하지 않고 빈 보정으로
시작한다. 미보정 CameraInfo는 실제 디코딩 크기와 `K[0]=0`을 갖는다. metric pose에
사용하지 않는다. 보정 도구는 각각 `/b2/front_camera/set_camera_info`와
`/b2/back_camera/set_camera_info`를 사용한다. private service는 같은 카메라 namespace로
연결한다. URL이 없으면 기본적으로 `~/.ros/camera_info/b2_front_camera.yaml`과
`~/.ros/camera_info/b2_back_camera.yaml`에 해당 이름으로 저장한다. `ROS_HOME`을
설정했다면 그 아래의 `camera_info/`를 확인한다. URL을 지정한 경우 그 파일에 저장한다.
보정 도구 프로세스에는 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`를 지정해 카메라 저장
서비스와 같은 RMW를 사용한다. 이 환경에서는 CycloneDDS 클라이언트의 저장 요청이
시간 초과됐다. 영상 토픽의 CycloneDDS 수신은 별도 통신 시험으로 확인한다.
저장된 각 실측 YAML을 확인한 뒤 `require_calibration:=true camera_info_url:=…`로 재실행한다.

## 검증 범위

현재 SDK archive와 CycloneDDS runtime은 Linux ELF이다. **macOS에서는 실기 노드를
링크하거나 실행할 수 없다.** `-DB2_CAMERA_BUILD_DRIVER=OFF`는 SDK와
camera_info_manager 없이 영상 디코딩·보정 계약·launch 테스트만 빌드한다. 실기 지원을
의미하는 대체 카메라 구현은 제공하지 않는다.

```bash
colcon build --base-paths src/shalom --packages-select b2_camera \
  --cmake-args -DB2_CAMERA_BUILD_DRIVER=OFF -DBUILD_TESTING=ON
colcon test --packages-select b2_camera
```

테스트는 합성 JPEG의 BGR8 디코딩·짝지어진 timestamp/frame, 실측 파라미터 보존,
크기 불일치·잘못된 보정·손상된 샘플 거절, 전후방 launch 분리와 입력 검증을 확인한다.
실제 B2 영상 API의 동작, 프레임 속도·지연과 정상 종료는 실기 검증 대상이다.

드라이버를 빌드하면 실제 `b2_camera_node` 두 개와 기존 SDK의 전·후방 VideoClient를
로컬 가짜 영상 RPC 서버에 연결하는 `sdk_video_runtime` 시험도 실행한다.
SDK domain 112와 NIC `lo`, ROS domain 220을 사용하며 보행 제어 채널은 만들지 않는다.
서로 다른 JPEG·intrinsic·optical frame을 확인하고 한쪽의 손상·빈 샘플·RPC 오류·크기
불일치·응답 누락 중에도 다른 쪽의 수신과 이후 복구를 검사한다. 각 프로세스의 정상 종료도 확인한다.
5/30 Hz 요청 상한 두 조건을 3회 반복해 통과했다. 이 PC에서 기록한 SDK 수신 완료→ROS
구독 지연의 95백분위는 약 0.14초, 최댓값은 0.15초 미만이었다. 합성 JPEG·로컬 통신
결과이며 실제 노출→검출 지연이 아니다. 높은 요청 상한은 위 전송 예산에 따라 낮아진다.

Linux에서 `B2_CAMERA_RUN_DDS_INTEROP=1 colcon test --packages-select b2_camera`로 별도 domain에서
Fast DDS Image·CameraInfo 발행과 프로젝트 CycloneDDS loopback 수신을 검증한다.
SDK·카메라·로봇을 시작하지 않는 통신 시험이며 양쪽 RMW와 `rclpy`가 필요하다.
Fast DDS 환경변수와 interface whitelist는
[공식 2.14 문서](https://fast-dds.docs.eprosima.com/en/v2.14.5/fastdds/env_vars/env_vars.html)와
[transport 설정](https://fast-dds.docs.eprosima.com/en/v2.14.5/fastdds/transport/whitelist.html)을 따른다.

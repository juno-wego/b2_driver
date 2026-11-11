# B2 전·후면 카메라 캘리브레이션

공식 ROS `camera_calibration`의 단안 GUI·체커보드 검출·최적화기를 사용한다.
`b2_camera`에서 선택한 raw 영상 하나를 받아 전면과 후면을 각각 보정한다.
렌즈 내부 파라미터 K·D·R·P를 구하며, URDF 카메라 장착 TF나 마커→도킹 목표는 별도로 측정한다.

| 구성 | 역할 |
|---|---|
| `camera_calibration` | 공식 OpenCV 체커보드 보정·GUI·샘플 선택 |
| `b2_camera_calibration` | 카메라 선택, 초기 모델, 토픽·저장 경로·GUI 종료 연결 |
| `b2_camera` | SDK raw 영상·CameraInfo·`set_camera_info` 저장 서비스 |
| `camera_info_manager`·`camera_calibration_parsers` | CameraInfo 파일 저장·읽기 |

두 카메라는 서로 반대 방향을 보므로 이 절차는 **각각의 단안 보정**이다.
앞뒤 영상을 하나의 stereo 보정으로 묶지 않는다. 기본 모델은 B2 참고값과 같은
`fisheye`이며, 실제 렌즈에 맞춰 `camera_model:=pinhole`도 명시할 수 있다.
선택한 모델은 첫 raw 프레임 처리 전부터 적용된다.
fisheye는 공식 도구의 체커보드 자세 재추정·조건수 검사 옵션을 사용한다.
skew를 0으로 강제하지 않는다. 조건수 오류가 나면 거리·기울기·화면 위치가 다양한
선명한 표본을 다시 수집한다. 여기서 재추정하는 자세는 체커보드 관측 자세다.

## 준비와 설치

이 PC는 ROS 2 Jazzy·Linux에서 공식 `image_pipeline` **5.0.13**의
`camera_calibration`을 workspace overlay에 설치했다. 사용 commit은
`6c3df3099bc7b1ec92215719215c8eefd0d3aa69`이며 외부 소스는 `.deps/perception/`에 둔다.
[공용 perception 목록](../../../l2_perception/apriltag_perception/docs/perception_dependencies.repos)으로 재현한다.
시스템에 적절한 `camera_calibration`이 이미 있으면 같은 소스를 중복 설치할 필요가 없다.

```bash
cd ~/shalom_ws
source /opt/ros/jazzy/setup.bash
mkdir -p .deps/perception
vcs import .deps/perception < src/shalom/robot/l2_perception/apriltag_perception/docs/perception_dependencies.repos
colcon build --base-paths .deps/perception/image_pipeline/camera_calibration \
  --packages-select camera_calibration --symlink-install
```

공식 패키지의 Python 의존성은 `python3-semver`다. 이 PC에서는 시스템 Python을
변경하지 않고 아래 순수 Python 의존성을 해당 overlay의 Python 경로에 설치했다.
`cv_bridge`, `image_geometry`, `message_filters`, OpenCV·NumPy·ROS Python도 필요하다.

```bash
/usr/bin/python3 -m pip install --no-deps \
  --target=install/camera_calibration/lib/python3.12/site-packages \
  -r src/shalom/robot/third_party/b2_driver/b2_camera_calibration/docs/python_requirements.txt
source install/setup.bash
colcon build --base-paths src/shalom --packages-select b2_camera b2_camera_calibration \
  --symlink-install --cmake-args -DB2_CAMERA_BUILD_DRIVER=ON -DBUILD_TESTING=ON
source install/setup.bash
```

## 실행

실물 체커보드의 **내부 코너 개수(열×행)**와 **한 칸 변 길이(m)**를 측정한다.
검출 태그 ID·AprilTag 변 길이와 다른 값이다. 아래 예시 `8x6`, `0.025`는
8×6 내부 코너·25 mm 정사각형인 보드에만 해당하며, 보유 보드 값으로 바꾼다.
잘못된 값·미입력·0·NaN·Inf는 카메라 시작 전에 거절한다.

이미 같은 카메라의 SDK publisher를 실행 중이면 먼저 그 실행기를 종료한 뒤,
별도 터미널에서 다음을 실행한다. launch가 선택한 카메라 하나와 보정 GUI를 함께 연다.

```bash
ros2 launch b2_camera_calibration calibrate.launch.py \
  network_interface:="$B2_NIC" camera_side:=front \
  board_size:=8x6 square_size_m:=0.025

ros2 launch b2_camera_calibration calibrate.launch.py \
  network_interface:="$B2_NIC" camera_side:=back \
  board_size:=8x6 square_size_m:=0.025
```

GUI 제목에 `b2_front_camera` 또는 `b2_back_camera`와 모델이 표시된다.
전면 작업을 마친 뒤 후면을 진행하면 된다. `output_root` 기본값은
B2 드라이버의 `b2_camera/config/calibration/`이며 다른 절대 경로로 지정할 수 있다.
symlink install에서는 원본 드라이버 디렉터리에 저장한다. 읽기 전용 설치에서는
`output_root`를 쓰기 가능한 경로로 지정한다.

| 인자 | 의미 |
|---|---|
| `network_interface` | 필수 B2 Ethernet NIC |
| `camera_side` | `front` 또는 `back`; 기본 `front` |
| `board_size` | 필수 내부 코너 개수 `NxM`; 각 축 3 이상 |
| `square_size_m` | 필수 실측 한 칸 길이(m) |
| `camera_model` | 기본 `fisheye`; 선택적으로 `pinhole` |
| `output_root` | 결과 YAML·샘플 아카이브를 보관할 root |
| `publish_rate_hz` | SDK polling 상한, 기본 5 Hz |
| `rpc_timeout_s`·`dds_domain` | 기존 B2 영상 SDK 설정, 기본 0.5초·0 |

카메라는 `require_calibration=false`, 빈 `camera_info_url`로 시작한다.
코너스 참고 파일을 로드하거나 덮어쓰지 않는다. GUI에는 raw 영상이 연결되며
rectifier·AprilTag 검출·도킹·보행 제어는 실행하지 않는다.
GUI와 카메라의 저장 서비스는 모두 해당 프로세스에 지정한 Fast DDS를 사용한다.
GUI 종료·실패·COMMIT 완료 시 같은 launch의 카메라도 종료한다.

## 촬영·저장 순서

1. 보드 전체가 보이게 중앙·좌우·상하·모서리와 서로 다른 거리·기울기를 채운다.
   보드 평면을 휘게 잡지 않고 이동 뒤 멈춰 선명한 프레임을 수집한다.
2. GUI의 X·Y·Size·Skew 분포가 충분해지면 **CALIBRATE**를 누른다.
   기본 fisheye에서 결과 모델은 `equidistant`와 D 4개다.
3. 보정 영상의 직선·모서리와 출력된 오차를 확인한 뒤 **SAVE**를 누른다.
   해당 카메라 이름의 YAML과 별도 세션의 원본 영상·보정 아카이브가 저장된다.
4. **COMMIT**은 선택한 카메라의 `set_camera_info` 서비스로 결과를 전달한다.
   launch가 지정한 `ROS_HOME=output_root` 아래 같은 카메라 이름의 YAML에 저장한다.
   성공하면 GUI와 선택한 SDK publisher가 종료된다.

COMMIT 요청은 5초로 제한하며 저장 거절·응답 유실 때 GUI를 유지한다.
이때 SAVE로 결과를 보존한 뒤 저장 경로·선택한 카메라 서비스를 확인할 수 있다.

```text
<output_root>/
  camera_info/
    b2_front_camera.yaml
    b2_back_camera.yaml
  datasets/
    b2_front_camera/<UTC시각_세션ID>/calibrationdata.tar.gz
    b2_front_camera/<UTC시각_세션ID>/session.json
    b2_back_camera/<UTC시각_세션ID>/calibrationdata.tar.gz
    b2_back_camera/<UTC시각_세션ID>/session.json
```

SAVE는 해당 카메라의 최신 YAML을 갱신하고, 이전 세션 아카이브는 유지한다.
각 아카이브에는 공식 도구 형식의 `ost.yaml`·`ost.txt`와 수집 영상이 포함된다.
공유 `/tmp/calibrationdata.tar.gz`를 앞뒤가 덮어쓰는 방식은 사용하지 않는다.

## 보정 후 사용

아래 전면 예시와 같은 방식으로 후면은 별도 `b2_back_camera.yaml`을 지정한다.
GUI 창·SDK가 종료된 다음 보정된 운용 모드로 다시 시작한다.

```bash
ros2 launch b2_camera camera.launch.py \
  network_interface:="$B2_NIC" camera_side:=front \
  camera_info_url:=package://b2_camera/config/calibration/camera_info/b2_front_camera.yaml
```

이중 도킹의 `sources.front.camera_info_url`과 `sources.back.camera_info_url`에도
해당 파일을 각각 지정한다. 파일 이름과 `camera_name`은
[카메라 파일 정리](../b2_camera/config/calibration/README.md)를 따른다.
전면 `f_oc_link`·후면 `r_oc_link` 장착 TF는 URDF에서 별도로 확인한다.
실측 보정 후에도 도킹의 절대 오차·영상 지연·제동과 충전 판정은 실기로 검증한다.

## 확인 범위

실물 보드를 촬영한 캘리브레이션 결과를 제공하는 패키지는 아니다.
시험은 launch의 카메라·이름·raw 토픽·저장 경로 분리와 입력 거절, 공식 fisheye
최적화기의 합성 코너 보정, GUI의 모델 선택·관측 연결·결과 저장을 검사한다.
실제 SDK 드라이버를 SDK domain 115/NIC `lo`에서 실행해 GUI의 COMMIT이 앞뒤별
`set_camera_info`에 저장되는지도 확인한다. 보행 SDK나 실물 B2에는 연결하지 않는다.
관련 공식 구현은 [ROS image_pipeline](https://github.com/ros-perception/image_pipeline/tree/6c3df3099bc7b1ec92215719215c8eefd0d3aa69/camera_calibration)이다.

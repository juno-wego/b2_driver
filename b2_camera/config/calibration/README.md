# B2 전·후면 카메라 보정 파일

전면·후면은 각각 캘리브레이션하고 별도 YAML로 관리한다. 두 카메라의 렌즈 내부
파라미터 K·D·R·P를 공유하지 않는다. 실행 시에도 각자의 `camera_info_url`을 지정한다.

| 구분 | 전면 | 후면 |
|---|---|---|
| `camera_side` | `front` | `back` |
| 실측 결과 파일명 | `b2_front_camera.yaml` | `b2_back_camera.yaml` |
| YAML `camera_name` | `b2_front_camera` | `b2_back_camera` |
| 영상·CameraInfo namespace | `/b2/front_camera` | `/b2/back_camera` |
| URDF optical frame | `f_oc_link` | `r_oc_link` |
| 보정 저장 서비스 | `/b2/front_camera/set_camera_info` | `/b2/back_camera/set_camera_info` |

실측 파일의 기본 저장 위치는 `~/.ros/camera_info/`이며 `ROS_HOME`을 지정하면 그 아래의
`camera_info/`를 사용한다. 파일 형식은
[전면 양식](b2_front_camera.example.yaml)·[후면 양식](b2_back_camera.example.yaml)으로
분리했다. 양식의 `null`은 미측정값이므로 실행에 사용하지 않는다. 보정 도구의 실제
결과를 각 파일로 저장하고 해당 `camera_name`을 맞춘다. 해상도·왜곡 모델도 실제
카메라의 보정 결과를 사용한다.

실측 작업은 [b2_camera_calibration](../../../b2_camera_calibration/README.md)의
`calibrate.launch.py`로 진행한다. 이 실행기의 기본 결과 경로는
드라이버가 소유한 `b2_camera/config/calibration/camera_info/`이며 앞뒤 파일 이름은
위 표와 같다. symlink install에서는 원본 디렉터리에 저장하며, 새 파일은 드라이버를
다시 빌드한 뒤 `package://b2_camera/config/calibration/camera_info/...` URL로 사용한다.

보정 저장 도구 프로세스에는 카메라 서비스 서버와 같은
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`를 지정한다. 이 환경의 `set_camera_info` 시험에서는
CycloneDDS 클라이언트의 요청이 시간 초과됐고 Fast DDS 클라이언트의 저장은 성공했다.
Image·CameraInfo 토픽의 CycloneDDS 수신과 보정 저장 서비스의 통신 조건은 각각 확인한다.

드라이버는 `camera_side`에 맞는 이름과 파일의 `camera_name`이 다르면 SDK 연결 전에
실행을 거절한다. 이름이 다를 때 경고만 출력하고 다른 카메라의 K/D를 쓰지 않는다.
`camera_name`은 YAML·보정 저장 시 사용하는 식별자다. ROS `CameraInfo` 메시지 자체에는
이름 필드가 없으므로 토픽 namespace와 `header.frame_id`로 전·후면을 구분한다.

## 현재 보관한 코너스 전면 참고값

현재는 `corners` 전면 값을 임시 참고 보정으로 보존한다. 추후 이 B2에서 측정한
전면·후면 YAML을 각각 지정하며, 출처 원본과 참고 프로필은 비교용으로 그대로 남긴다.

| 항목 | 가져온 값 |
|---|---|
| 해상도·모델 | 1920×1080, `equidistant` fisheye |
| `fx`, `fy` | 709.03283, 719.53793 px |
| `cx`, `cy` | 960.18835, 506.59977 px |
| skew | 5.18641 px |
| `D` | `[0.387385, 0.107582, -0.100004, -0.01427]` |
| `R` | 항등행렬 |

실행 가능한 [b2_front_1920x1080.yaml](b2_front_1920x1080.yaml)은 코너스 전면 참고 프로필이다.
원본 파일은 [corners](corners/manifest.json) 아래에 byte 그대로 추가 보존했다.

| 원본 보관 파일 | 용도 |
|---|---|
| `corners/b2_camera_info_raw.yaml` | 원래 K·D·R·P와 카메라 이름 |
| `corners/b2_camera_info_rectified.yaml` | 기존 프로젝트의 보정 후 메타데이터 참고용 |
| `corners/ost.yaml`, `corners/ost.txt` | ROS calibrator 결과 두 형식 |
| `corners/260402_1312.txt` | 같은 보정값의 세션 로그 |
| `corners/manifest.json` | 커밋, 원본 URL, 파일별 크기·SHA-256·Git blob |

`corners`의 후면 launch도 기본적으로 같은 raw 파일을 지정하지만, 별도로 측정한
후면 보정 결과는 확인되지 않았다. 전면 값을 후면 실측값으로 복제하지 않는다.
원본 캘리브레이션 영상 묶음은 출처 경로만 manifest에 기록했다.

`b2_front_1920x1080.yaml`은 제공된
[Wego-Robot-Engeering-Team/corners의 원본 YAML](https://github.com/Wego-Robot-Engeering-Team/corners/blob/cc265b5dd849af219f1f94f6a28e35cf9ce24719/sensors/b2_camera/b2_camera_bridge/config/b2_camera_info_raw.yaml)에서
K·D·R·P와 해상도를 그대로 가져온 파일이다. 전방 카메라의 1920×1080 영상에 대응하며,
후방 카메라의 보정값이나 카메라 장착 TF를 포함하지 않는다.

- 원본 저장소: `Wego-Robot-Engeering-Team/corners`
- 원본 commit: `cc265b5dd849af219f1f94f6a28e35cf9ce24719`
- 원본 경로: `sensors/b2_camera/b2_camera_bridge/config/b2_camera_info_raw.yaml`
- Git blob: `9fef242e5ac3fb4bea696b139e3f6f14fab51d0c`
- SHA-256: `d636ca5e201b2bed6596eba50aad651f35f260ef3504ad5add78ecac2c5e6d89`

같은 저장소의 `sensors/b2_camera/b2_camera_calibration/data/fisheye/260402_1312.txt`
로그에 같은 fisheye K·D·R·P가 기록되어 있다. 이 로그의 후방 RPC 오류는 후방
보정 측정을 뜻하지 않는다. 보정값의 소수 자릿수도 유지했다. `camera_name`만
원본 `b2_camera`에서 드라이버 이름 `b2_front_camera`로 바꿔 로드 시 이름 경고를
방지했다. 현재 드라이버에서는 이 이름이 맞아야 로드할 수 있다. 위 hash는 원본
588바이트를 식별하며, 설치 파일의 hash와 유일한 metadata
변경은 `source.json`에 기록했다. 테스트가 metadata를 복원해 원본 Git blob도 확인한다.
기계가 읽을 출처 정보는 [source.json](source.json)에 있다.

## 실측 전·후면 파일 선택

보정 후 각 파일을 지정해 별도 프로세스로 실행한다.

```bash
ros2 launch b2_camera camera.launch.py \
  network_interface:="$B2_NIC" camera_side:=front \
  camera_info_url:="file://$HOME/.ros/camera_info/b2_front_camera.yaml"

ros2 launch b2_camera camera.launch.py \
  network_interface:="$B2_NIC" camera_side:=back \
  camera_info_url:="file://$HOME/.ros/camera_info/b2_back_camera.yaml"
```

이중 카메라 관측 설정의 `sources.front.camera_info_url`과
`sources.back.camera_info_url`에도 위의 서로 다른 파일 URL을 넣는다. 관측 설정 YAML
한 개가 두 보정 파일의 경로를 묶는 것이며 K/D를 하나의 파일로 공유하는 구조가 아니다.

## 코너스 참고값을 명시적으로 선택하기

```bash
ros2 launch b2_camera camera.launch.py \
  network_interface:=<B2에_연결된_NIC> camera_side:=front \
  camera_info_url:=package://b2_camera/config/calibration/b2_front_1920x1080.yaml
```

일반 camera launch의 `camera_info_url` 기본값은 계속 비어 있다. 후방 또는 다른
카메라에 이 파일을 자동 적용하지 않는다. 카메라 serial은 원본에 없으므로 설치
카메라와 해상도가 이 측정 대상과 같은지 확인해야 한다. 영상 크기가 다르면
드라이버가 프레임을 버리며 보정값을 자동으로 확대·축소하지 않는다.

## Raw와 L2 보정의 경계

원본은 `equidistant` 모델의 4개 D와 `K[0,1]=P[0,1]=5.18641` skew를 포함한다.
L1은 raw 영상과 이 값을 보존한다. 영상 보정은 L2에서 한 번 수행하고,
보정 후 projection은 AprilTag PnP가 사용하는 zero-skew pinhole 좌표계로 만든다.
입력 skew는 fisheye remap의 원본 x 좌표에도 반영해야 한다.

옛 `b2_camera_info_rectified.yaml`의 0인 D를 raw 카메라 보정으로 사용하지 않는다.
이미 보정된 영상을 다시 fisheye 보정하지 않으며, 옛 TF 기본값도 복사하지 않는다.

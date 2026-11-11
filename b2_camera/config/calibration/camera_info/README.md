# 이 B2에서 측정한 카메라 보정

실측 결과를 `b2_front_camera.yaml`, `b2_back_camera.yaml`로 각각 저장한다.
각 YAML의 `camera_name`도 각각 `b2_front_camera`, `b2_back_camera`여야 한다.
아직 실물이 없으므로 실행 가능한 실측값을 만들어 넣지 않는다.

상위 `b2_driver` 저장소의 `b2_camera_calibration` 도구가 SAVE/COMMIT으로 이 경로를
사용한다. symlink install에서는 원본 드라이버의 보정 디렉터리에 저장한다.
새 YAML을 저장한 뒤 `b2_camera`를 다시 빌드하면 다음 URL로 운용할 수 있다.

```text
package://b2_camera/config/calibration/camera_info/b2_front_camera.yaml
package://b2_camera/config/calibration/camera_info/b2_back_camera.yaml
```

읽기 전용 설치 환경에서는 캘리브레이션 도구의 `output_root`를 쓰기 가능한 경로로
지정하고, 실측 YAML을 배포용 드라이버에 반영한다. `camera_info_url`로 외부 파일을
명시하는 방식도 계속 지원한다. `corners/`는 출처 보존용 참고값이다.

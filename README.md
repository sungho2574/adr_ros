# adr_ros

Jetson 실기체 준비용 ROS 2 워크스페이스. 카메라, GateNet, 배터리 HUD, 영상 송출, rosbag을 실행한다. Betaflight 기체용 정책 추론을 별도로 켤 수 있다.

## Jetson에서 실행

ROS 2와 JetPack의 OpenCV, NumPy, PyYAML, PyGObject/GStreamer, Argus를 사용한다. GateNet 추론에는 해당 JetPack에서 동작하는 TensorRT/PyCUDA 또는 ONNX Runtime이 필요하다. 실시간 송출 시 MediaMTX와 GStreamer H.264/RTSP 플러그인도 설치한다.

선택적으로 Jetson에서 GateNet TensorRT 엔진을 만든다. 엔진은 빌드 전에 이 위치에 두거나, 외부 절대 경로를 `perception.engine_file`에 지정한다.

```bash
/usr/src/tensorrt/bin/trtexec --onnx=src/adr_perception/configs/gatenet_240x320.onnx \
  --saveEngine=src/adr_perception/configs/gatenet_240x320_fp16.plan --fp16
```

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch adr_bringup camera_test.launch.py view:=false
```

카메라 테스트에서 `view:=true`로 rqt 영상 뷰어를 띄울 수 있다. 테스트를 종료한 뒤 실기체 설정으로 HUD 조회, 송출과 녹화를 함께 실행한다.

```bash
ros2 launch adr_bringup real.launch.py
```

기본 파라미터는 [real.yaml](src/adr_bringup/config/real.yaml)에 있다. `config:=/path/to/real.yaml`로 덮어쓸 수 있다. IMX219은 mode 3, 1640×1232, 30 fps다. 카메라 노드에서 320×240 BGR로 하드웨어 축소한 다음 발행한다. 노출 4 ms와 아날로그/ISP 게인 1은 GateNet 인계 설정을 따른다.

## Betaflight UART 텔레메트리

`adr_betaflight`가 UART와 MSP 프로토콜을 담당한다. `adr_control`은 ROS IMU를 구독하고, `adr_viz`는 ROS 배터리를 구독한다. 실기체 런치는 `real.yaml`의 `betaflight` 설정으로 노드를 시작한다. 카메라만 점검할 때는 `camera_test.launch.py` 또는 `real.launch.py msp:=false`를 쓰면 된다.

Jetson UART는 **`/dev/ttyTHS1`, 500000 baud, 8N1**이다. FC에서 실제 연결한 하드웨어 UART의 MSP를 켜고 500000 baud를 맞춘다. 두 장치의 UART 레벨을 확인한 뒤 TX↔RX와 공통 GND를 연결한다. [Betaflight Ports 설정](https://betaflight.com/docs/wiki/app/ports-tab) · [Betaflight baud 목록](https://github.com/betaflight/betaflight/blob/4.5.2/src/main/io/serial.c#L108-L110)

텔레메트리만 단독으로 실행할 수 있다. Jetson의 시스템 Python에 `python3-serial`이 필요하다.

```bash
ros2 run adr_betaflight msp_telemetry --ros-args \
  -p port:=/dev/ttyTHS1 -p baud:=500000 -p acc_lsb_per_g:=512.0
ros2 topic hz /adr/telemetry/imu
ros2 topic echo /adr/telemetry/battery --qos-reliability best_effort
```

IMU와 자세는 하나의 MSP 요청으로 묶어 **100 Hz**를 목표로 발행한다. 배터리 조회는 **2 Hz**를 목표로 같은 요청에 추가한다. 토픽 타임스탬프는 Jetson의 응답 수신 시각이다. 첫 가속도 raw 샘플을 노드 로그에 남긴다. 기체가 수평 정지 상태일 때 `Z ≈ +512 LSB`와 ROS의 `Z ≈ +9.81 m/s²`를 확인한다. yaw는 FC 헤딩에서 변환하므로 절대 방위 기준의 정확도는 FC 설정에 달려 있다. [Betaflight MSP 응답 형식](https://github.com/betaflight/betaflight/blob/4.5.2/src/main/msp/msp.c#L2191-L2224)

배터리 잔량은 FC에 용량과 전류계가 설정돼 있으면 사용량으로 추정하고, 그 외에는 FC의 셀 전압 범위로 추정한다. 계산에 필요한 설정이 없으면 미측정으로 표시한다. [Betaflight 배터리 잔량 계산](https://github.com/betaflight/betaflight/blob/2025.12.1/src/main/sensors/battery.c#L457-L469)

## 토픽

| 토픽 | 내용 |
|---|---|
| `/adr/camera/image_raw` | 원본 화각을 320×240으로 줄인 BGR 프레임 |
| `/adr/camera/camera_info` | 위 영상의 내부 파라미터와 왜곡 계수 |
| `/adr/perception/mask` | 정책 입력용 64×64 `mono8` 마스크, 픽셀값 0 또는 255 |
| `/adr/perception/mask_full` | 왜곡 보정 좌표계의 320×240 `mono8` 마스크 |
| `/adr/perception/image_rect` | 마스크와 픽셀 좌표가 같은 320×240 보정 영상 |
| `/adr/perception/latency_ms` | 카메라 타임스탬프에서 마스크 발행 직전까지 걸린 시간 |
| `/adr/telemetry/imu` | Betaflight 각속도·가속도·자세를 담은 `sensor_msgs/Imu` |
| `/adr/telemetry/battery` | 전압, 전류와 추정 잔량을 담은 `sensor_msgs/BatteryState` |
| `/adr/monitor/image`, `/adr/monitor/image/compressed` | GateNet 마스크와 하단 배터리 전압·잔량을 표시한 640×480 HUD |
| `/adr/monitor/status` | 배터리 값과 수신 상태 |

영상과 마스크는 best effort를 사용한다. 정책용 마스크 구독은 depth 1이며 카메라 타임스탬프를 보존한다. Argus 버퍼 PTS를 쓸 수 없으면 카메라 도착 시각을 사용한다. HUD는 동일한 타임스탬프의 영상과 마스크를 사용한다.

HUD 기본 배터리 입력은 `/adr/telemetry/battery`의 `sensor_msgs/BatteryState`다. 전압과 0~1 범위의 잔량을 표시한다. PX4 DDS를 쓰는 경우 `px4_msgs`를 준비하고 `viz.battery_source: px4`로 설정한다. 배터리 발행이 없거나 데이터가 오래되면 빈 값을 표시한다.

실기체 런치는 기본으로 `flight_logs/`에 rosbag을 만들고 종료 시 마감한다. `record:=false`로 끄고, `log_dir:=...`로 경로를 바꾼다. PX4의 ARM/DISARM에 맞추려면 `record_trigger:=px4_arm`을 지정한다. GateNet 인계 형식의 입력 JPEG·마스크 PNG·CSV가 필요하면 `perception.record: true`를 설정한다. MediaMTX가 PATH에 있으면 로컬 HUD 스트림을 시작한다. `stream_url:=`로 끄거나 RTSP 주소를 지정할 수 있다.

## 100 Hz 정책 추론

`adr_control`은 `deploy/onboard.py`의 NumPy 정책과 게이트 전환 로직을 사용한다. `bundle_dir`에는 `policy_meta.json`과 `policy_weights.npz`가 필요하다. 첨부한 `deploy`는 export 소스만 포함한다. 번들의 시작 게이트와 전환 조건을 읽고, 같은 마스크를 재사용하는 tick에서는 CNN 결과를 캐시한다.

```bash
ros2 launch adr_bringup real.launch.py \
  policy_bundle:=/absolute/path/to/deploy_bundle policy_bench_imu:=true
ros2 topic echo /adr/control/status --qos-reliability best_effort
```

`policy_bench_imu:=true`는 정지·수평 IMU 관측값을 넣어 GateNet과 정책을 벤치에서 실행한다. 실측 IMU를 쓸 때는 이 옵션을 빼고, 카메라 스택과 같은 ROS 도메인에 `/adr/telemetry/imu`를 발행한다. 정책 노드만 실행할 때는 `ros2 run adr_control policy_node --ros-args -p bundle_dir:=/absolute/path/to/deploy_bundle`를 사용한다.

| 토픽 | 내용 |
|---|---|
| `/adr/perception/mask` | 카메라 타임스탬프가 있는 64×64 GateNet 마스크. 최신 값을 100 Hz 루프에서 재사용 |
| `/adr/telemetry/imu` | 자세·각속도·가속도가 채워진 `sensor_msgs/Imu`. 자세는 ENU 월드 기준의 기체 FLU 쿼터니언, 각속도는 rad/s, 가속도는 정지 시 +Z에 약 9.81 m/s²가 들어오는 기체 FLU 값 |
| `/adr/control/action` | `Float32MultiArray`에 담은 4개 정규화 액션. 순서는 throttle, roll, pitch, yaw rate |
| `/adr/control/status` | 1초마다 발행하는 JSON. 실측 추론 빈도, 10 ms 이상 걸린 횟수, 입력 나이, 게이트 인덱스와 `p_g` 포함 |

입출력은 best effort, depth 1이다. 카메라와 IMU의 타임스탬프는 노드와 같은 ROS 시계를 사용한다. 기본 입력 유효기간은 마스크 150 ms, IMU 50 ms다. 두 값은 단독 노드에서 `mask_timeout_s`, `imu_timeout_s` 파라미터로 설정한다. 입력이 만료되면 액션 발행을 멈추고 정책 메모리와 게이트 인덱스를 초기화한다. 벤치 모드는 새 입력으로 추론을 재개한다. 실측 IMU 모드에서 실행 중 입력이 만료되면 노드 재시작을 요구한다.

Betaflight UART는 MSP 조회 전용이다. 이후 RC 제어 연동 단계에서 입력이 끊기면 조종자에게 제어권을 돌려주고 지상에서만 정책을 다시 시작하는 조건을 적용한다.

복사해 온 ArUco, 정밀착륙 컨트롤러, SITL/HITL 전용 파일은 `legacy/precision_landing/`에 참고용으로 보관한다.

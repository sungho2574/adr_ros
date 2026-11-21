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

기본 파라미터는 [real.yaml](src/adr_bringup/config/real.yaml)에 있다. `config:=/path/to/real.yaml`로 덮어쓸 수 있다. IMX219은 mode 3, 1640×1232, 30 fps다. 카메라 노드에서 320×240 BGR로 하드웨어 축소한 다음 발행한다. 실기체 런치는 기본으로 Argus 자동 노출과 게인을 사용한다. `auto_exposure:=false`를 지정하면 YAML의 노출·게인 설정을 사용한다. 기본 YAML의 고정 노출은 8.33 ms이며 아날로그/ISP 게인은 1이다.

## 영상 송출 스모크 테스트

### 카메라 단독

카메라, GateNet, HUD와 RTSP 송출을 점검한다. 아래 FC 런치 예시의 환경 설정을 먼저 적용한다.

```bash
ros2 launch adr_bringup smoke_test.launch.py
```

기본 카메라 모드는 실기체와 같은 mode 3이며, 두 스모크 테스트 모두 기본으로 Argus 자동 노출과 게인을 사용한다. 8.33 ms 고정 노출을 쓰려면 `auto_exposure:=false`를 지정한다.

실기체 설정으로 카메라, GateNet, HUD, 스트리머를 띄우고 rosbag을 기록한다. 녹화는 `record:=false`로 끄거나 `log_dir:=...`로 저장 경로를 바꿀 수 있다. Flight Analyzer 목록에는 스모크 기록이 `SMOKE`로 표시된다.

기본 송출은 로컬 MediaMTX를 사용한다. 다른 기기에서 `http://JETSON_IP:8889/hud` 또는 `rtsp://JETSON_IP:8554/hud`로 확인할 수 있다. 기존 RTSP 서버를 쓰려면 `stream_url:=rtsp://HOST:8554/hud`를 지정한다.

### 카메라와 FC 연결

카메라 파이프라인과 Betaflight UART 텔레메트리를 함께 점검한다. HUD에서 FC 연결 상태와 자세를 확인할 수 있다.

젯슨의 새 터미널에서 아래 블록을 실행한다. 워크스페이스에 설치한 MediaMTX·GStreamer 플러그인과 `~/gatenet_ws`의 PyCUDA를 사용한다.

```bash
cd /home/inair/Workspace/adr_ros
source /opt/ros/humble/setup.bash
source install/setup.bash

export ADR_ROOT="$PWD"
gst_lib="$PWD/.deps/smoke/gst/usr/lib/aarch64-linux-gnu"
export LD_LIBRARY_PATH="$gst_lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export GST_PLUGIN_PATH="$gst_lib/gstreamer-1.0${GST_PLUGIN_PATH:+:$GST_PLUGIN_PATH}"
export PYTHONPATH="$HOME/gatenet_ws/.venv/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export MTX_WEBRTC=no MTX_MOQ=no

ros2 launch adr_bringup smoke_test.launch.py auto_exposure:=true
```

이 명령으로 실행하면 VLC에서 `rtsp://JETSON_IP:8554/hud`를 연다.

FC 텔레메트리까지 점검하려면 위 환경 설정 후 아래 명령을 실행한다. `msp_port`에는 사용할 시리얼 장치 경로를 넣는다. `msp_baud`에는 통신 속도를 넣는다. 각 옵션을 생략하면 설정 파일의 해당 값을 사용한다.

```bash
ros2 launch adr_bringup fc_smoke_test.launch.py auto_exposure:=true msp_port:=/dev/ttyTHS1 msp_baud:=115200
```

## Betaflight UART 텔레메트리

`adr_betaflight`가 UART와 MSP 프로토콜을 담당한다. `adr_control`은 ROS IMU를 구독하고, `adr_viz`는 ROS 배터리를 구독한다. 실기체 런치는 `real.yaml`의 `betaflight` 설정으로 노드를 시작한다. 카메라만 점검할 때는 `camera_test.launch.py` 또는 `real.launch.py msp:=false`를 쓰면 된다.

Jetson UART는 **`/dev/ttyTHS1`, 500000 baud, 8N1**이다. FC에서 실제 연결한 하드웨어 UART의 MSP를 켜고 500000 baud를 맞춘다. 두 장치의 UART 레벨을 확인한 뒤 TX↔RX와 공통 GND를 연결한다. [Betaflight Ports 설정](https://betaflight.com/docs/wiki/app/ports-tab) · [Betaflight baud 목록](https://github.com/betaflight/betaflight/blob/4.5.2/src/main/io/serial.c#L108-L110)

텔레메트리만 단독으로 실행할 수 있다. Jetson의 시스템 Python에 `python3-serial`이 필요하다.

```bash
ros2 run adr_betaflight msp_telemetry --ros-args \
  -p port:=/dev/ttyTHS1 -p baud:=500000 -p acc_lsb_per_g:=512.0
ros2 topic hz /adr/telemetry/imu
ros2 topic echo /adr/telemetry/battery --qos-reliability best_effort
ros2 topic echo /adr/telemetry/rc --qos-reliability best_effort
```

IMU와 자세는 하나의 MSP 요청으로 묶어 **100 Hz**를 목표로 발행한다. 배터리 조회는 **2 Hz**를 목표로 같은 요청에 추가한다. 토픽 타임스탬프는 Jetson의 응답 수신 시각이다. 첫 가속도 raw 샘플을 노드 로그에 남긴다. 기체가 수평 정지 상태일 때 `Z ≈ +512 LSB`와 ROS의 `Z ≈ +9.81 m/s²`를 확인한다. yaw는 FC 헤딩에서 변환하므로 절대 방위 기준의 정확도는 FC 설정에 달려 있다. [Betaflight MSP 응답 형식](https://github.com/betaflight/betaflight/blob/4.5.2/src/main/msp/msp.c#L2191-L2224)

배터리 잔량은 FC에 용량과 전류계가 설정돼 있으면 사용량으로 추정하고, 그 외에는 FC의 셀 전압 범위로 추정한다. 계산에 필요한 설정이 없으면 미측정으로 표시한다. [Betaflight 배터리 잔량 계산](https://github.com/betaflight/betaflight/blob/2025.12.1/src/main/sensors/battery.c#L457-L469)

`MSP_RC`는 기본 **25 Hz**로 같은 MSP 요청에 추가하며, `real.launch.py`와 `fc_smoke_test.launch.py`의 rosbag에 기록된다. `betaflight.rc_hz`로 조절하고 `0.0`으로 조회를 끈다. 각 샘플에 ROS 수신 시각(`stamp_ns`), 마이크로초 단위의 스틱·AUX 값, 같은 요청에서 읽은 모드와 수신 상태를 담는다. 각 채널은 FC가 적용 중인 값으로, MSP가 덮어쓴 경우에는 정책 입력값, failsafe가 적용된 경우에는 FC의 대체값이 들어갈 수 있다. [Betaflight `MSP_RC` 구현](https://github.com/betaflight/betaflight/blob/2026.6.2/src/main/msp/msp.c#L1282-L1286)

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
| `/adr/telemetry/fc_status` | MSP 연결, FC 버전, ARM, MSP OVERRIDE, failsafe, 수신기 손실·FAILSAFE 스위치 플래그, ARM 차단 사유 목록과 비트마스크 |
| `/adr/telemetry/rc` | `std_msgs/String` JSON의 `roll_us`, `pitch_us`, `yaw_us`, `throttle_us`, `aux_us`(AUX1부터 순서대로), `armed`, `msp_override`, `failsafe`, `rx_loss`, `failsafe_switch` |
| `/adr/monitor/image`, `/adr/monitor/image/compressed` | GateNet 마스크와 하단 배터리 전압·잔량을 표시한 320×240 HUD |
| `/adr/monitor/status` | 배터리 값과 수신 상태 |

`fc_status.arming_disable_flags`는 활성 차단 사유의 Betaflight CLI 이름 목록이다. 빈 배열은 활성 사유 없음, `null`은 차단 상태 응답 없음이다. `arming_disable_mask`에 FC가 보낸 비트마스크를 함께 기록하고, 이름을 확인할 수 없는 비트는 `BIT_n`으로 표시한다. [Betaflight ARM 차단 이름](https://github.com/betaflight/betaflight/blob/2026.6.2/src/main/fc/runtime_config.c#L34-L65)

영상과 마스크는 best effort를 사용한다. 정책용 마스크 구독은 depth 1이며 카메라 타임스탬프를 보존한다. Argus 버퍼 PTS를 쓸 수 없으면 카메라 도착 시각을 사용한다. HUD는 동일한 타임스탬프의 영상과 마스크를 사용한다.

HUD 기본 배터리 입력은 `/adr/telemetry/battery`의 `sensor_msgs/BatteryState`다. 전압과 0~1 범위의 잔량을 표시한다. PX4 DDS를 쓰는 경우 `px4_msgs`를 준비하고 `viz.battery_source: px4`로 설정한다. 배터리 발행이 없거나 데이터가 오래되면 빈 값을 표시한다.

HUD 우상단에는 MSP 연결과 롤·피치·요 각도를 표시한다. 좌상단 상태에 맞춰 테두리는 `DISARM` 회색, `ARM` 주황색, `AUTO` 초록색으로 바뀐다. `policy_rc:=true`에서는 정책 명령이 FC에 전달되기 시작하면 `AUTO`로 바뀐다. 텔레메트리 모드에서는 FC가 ARM 상태에서 MSP OVERRIDE를 보고하면 `AUTO`가 표시된다. FC 상태가 끊기거나 0.5초 동안 갱신되지 않으면 빨간 테두리와 `NO STATUS`를 표시한다. [Betaflight MSP OVERRIDE](https://betaflight.com/docs/wiki/guides/current/MSP-Companion-Computer-Control)

실기체 런치는 기본으로 `flight_logs/`에 rosbag을 만들고 종료 시 마감한다. `record:=false`로 끄고, `log_dir:=...`로 경로를 바꾼다. PX4의 ARM/DISARM에 맞추려면 `record_trigger:=px4_arm`을 지정한다. GateNet 인계 형식의 입력 JPEG·마스크 PNG·CSV가 필요하면 `perception.record: true`를 설정한다. MediaMTX가 PATH에 있으면 로컬 HUD 스트림을 시작한다. `stream_url:=`로 끄거나 RTSP 주소를 지정할 수 있다.

## 100 Hz 정책 추론

`adr_control`은 `onboard.py`의 NumPy 정책과 게이트 전환 로직을 사용한다. `b6p53M` 번들(`team44up`, 학습 스텝 6,530,400)의 `policy_meta.json`과 `policy_weights.npz`는 `adr_control`과 함께 설치한다. 정책은 번들의 시작 게이트와 전환 조건을 읽고, 같은 마스크를 재사용하는 tick에서는 CNN 결과를 캐시한다.

```bash
ros2 launch adr_bringup real.launch.py \
  policy_bundle:=b6p53M policy_bench_imu:=true
ros2 topic echo /adr/control/status --qos-reliability best_effort
```

`policy_bench_imu:=true`는 정지·수평 IMU 관측값을 넣어 GateNet과 정책을 벤치에서 실행한다. 실측 IMU로 정책 추론만 하려면 `policy_bundle:=b6p53M policy_rc:=false`를 지정한다. 다른 내보내기 번들은 `policy_bundle:=/absolute/path/to/deploy_bundle`로 지정한다. 정책 노드만 실행할 때는 `ros2 run adr_control policy_node --ros-args -p bundle_dir:=/absolute/path/to/deploy_bundle`를 사용한다.

| 토픽 | 내용 |
|---|---|
| `/adr/perception/mask` | 카메라 타임스탬프가 있는 64×64 GateNet 마스크. 최신 값을 100 Hz 루프에서 재사용 |
| `/adr/telemetry/imu` | 자세·각속도·가속도가 채워진 `sensor_msgs/Imu`. 자세는 ENU 월드 기준의 기체 FLU 쿼터니언, 각속도는 rad/s, 가속도는 정지 시 +Z에 약 9.81 m/s²가 들어오는 기체 FLU 값 |
| `/adr/control/action` | `Float32MultiArray`에 담은 4개 정규화 액션. 순서는 throttle, roll, pitch, yaw rate |
| `/adr/control/rc_action` | `policy_rc:=true`에서 현재 인계 세션 ID, Jetson 단조 시계의 생성 시각, 4개 정규화 액션을 담은 JSON |
| `/adr/control/status` | 1초마다 발행하는 JSON. 실측 추론 빈도, 10 ms 이상 걸린 횟수, 입력 나이, 게이트 인덱스와 `p_g` 포함 |

입출력은 best effort, depth 1이다. 카메라와 IMU의 타임스탬프는 노드와 같은 ROS 시계를 사용한다. 기본 입력 유효기간은 마스크 150 ms, IMU 50 ms다. 두 값은 단독 노드에서 `mask_timeout_s`, `imu_timeout_s` 파라미터로 설정한다. 입력이 만료되면 액션 발행을 멈추고 정책 메모리와 게이트 인덱스를 초기화한다. 벤치 모드는 새 입력으로 추론을 재개한다. 실측 IMU 모드에서 실행 중 입력이 만료되면 노드 재시작을 요구한다.

## Betaflight 정책 제어

```bash
ros2 launch adr_bringup real.launch.py
ros2 topic echo /adr/telemetry/fc_status --qos-reliability best_effort
```

실기체 런치는 기본으로 `policy_rc`를 켜고 설치된 `b6p53M`을 쓴다. `adr_betaflight`는 FC가 Betaflight 2026.6 이상, 물리 수신기, AETR 맵과 ANGLE·MSP OVERRIDE 모드를 보고할 때 RC 경로를 연다. 텔레메트리와 영상만 보려면 `policy_rc:=false`를 지정한다. `msp:=false` 또는 `policy_bench_imu:=true`를 지정하면 `policy_rc` 기본값도 `false`가 된다.

FC 설정에서 `msp_override_channels_mask = 15`, `msp_override_failsafe = OFF`, `angle_limit = 55`, yaw 최대 스틱 각속도 약 229°/s를 확인한다. ARM과 MSP OVERRIDE는 물리 조종기의 AUX 스위치에 매핑한다. 100 Hz마다 텔레메트리 조회와 RC 쓰기가 각각 한 번씩 이뤄지므로 `serial_update_rate_hz = 500`에서 시작해 Jetson에서 IMU 및 정책 발행률을 측정한다. ROS 토픽 조회는 `real.yaml`과 같은 `ROS_DOMAIN_ID`에서 실행한다. [Betaflight MSP OVERRIDE](https://betaflight.com/docs/wiki/guides/current/MSP-Companion-Computer-Control)

지상에서 DISARM·OVERRIDE OFF 상태로 런치한 뒤, ARM·ANGLE 상태에서 조종자가 OVERRIDE를 켜면 정책 메모리와 게이트 순서를 G1부터 시작한다. 정상 인계가 이뤄지면 `fc_status.rc_state`가 `active`가 된다. 조종자가 스위치를 끄면 수동으로 돌아가며, 다음 인계는 다시 DISARM·OVERRIDE OFF를 거쳐야 한다. 마스크나 IMU가 만료되면 정책 발행이 멈추고, 새 액션이 50 ms 동안 도착하지 않으면 MSP 브리지가 수신기로 제어권을 넘긴다. UART 자체가 끊기면 FC가 마지막 MSP 프레임에서 약 300 ms 뒤 수신기로 복귀한다. 이 경우 지상에서 수동 복귀와 DISARM을 확인한 뒤 런치를 다시 시작한다. 첫 확인은 프로펠러를 제거한 상태에서 조종기 스위치와 UART 분리로 진행한다. [Betaflight RC 입력 만료 처리](https://github.com/betaflight/betaflight/blob/2026.6.2/src/main/rx/msp.c)

복사해 온 ArUco, 정밀착륙 컨트롤러, SITL/HITL 전용 파일은 `legacy/precision_landing/`에 참고용으로 보관한다.

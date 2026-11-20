# ADR Flight Analyzer

ROS 2 rosbag의 HUD 영상과 텔레메트리를 같은 시간축에서 다시 보는 도구다. 기존 정밀착륙 기록도 열 수 있다.

## 녹화

ROS Humble 환경에서 rosbag CLI와 SQLite 플러그인을 설치하고 프로젝트를 빌드한다.

```bash
sudo apt install ros-humble-ros2bag ros-humble-rosbag2-storage-default-plugins
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch adr_bringup real.launch.py
```

실기체 런치와 `smoke_test.launch.py`는 기본으로 시작부터 종료까지 bag을 기록한다. `record:=false`로 끌 수 있다. PX4 실기체 런치는 `record_trigger:=px4_arm`으로 ARM에서 시작하고 DISARM에서 마감할 수 있다.

기본 저장 경로는 `flight_logs/<날짜_시각>_<세션>_<프로필>/bag/`이다. `log_dir:=/path/to/flight_logs`로 변경할 수 있다. 세션의 `manifest.json`에 런치 프로필과 녹화 시작 시각을 저장한다. 기록 목록은 프로필에 따라 `REAL` 또는 `SMOKE` 태그를 표시한다. 이전 녹화는 세션 폴더 이름에서 프로필을 확인한다.

기록에 들어 있는 GateNet 지연·발행률, FC 연결·ARM·MSP OVERRIDE, IMU 자세·각속도·가속도, 배터리 전압·잔량·전류, 정책 액션·추론 시간을 기본 그래프로 표시한다. HUD 영상은 `/adr/monitor/image/compressed`에서 읽는다.

## 다시 보기

Jetson에서 기본 `flight_logs/`를 직접 읽거나, 완료된 기록 디렉터리를 통째로 맥의 `flight_logs/`에 복사해 열 수 있다. 뷰어 실행에는 ROS가 필요 없다. 아래 명령은 저장소 루트에서 실행한다.

```bash
python3 -m venv scripts/flight_analyzer/.venv
source scripts/flight_analyzer/.venv/bin/activate
python -m pip install -r scripts/flight_analyzer/requirements.txt
python scripts/flight_analyzer/app.py --log-dir flight_logs
```

같은 기기의 브라우저에서 <http://127.0.0.1:5050/>을 열면 최신 기록을 불러온다. 새로 복사한 기록은 **최신 기록 열기** 버튼으로 선택할 수 있다. Jetson에서 뷰어를 실행하고 맥에서 보려면 맥에서 `ssh -N -L 5050:127.0.0.1:5050 JETSON_USER@JETSON_IP`로 포워딩한 뒤 같은 주소를 연다. 그래프 라이브러리 uPlot은 브라우저에서 CDN으로 불러온다.

별도 로그 폴더나 단일 bag 디렉터리는 `--log-dir`로 열 수 있다. PX4 메시지 필드를 볼 때는 기록과 같은 `px4_msgs` 서브모듈을 초기화하거나 `--px4-msg-dir /path/to/px4_msgs/msg`를 지정한다. HUD와 모니터 상태 그래프는 표준 ROS 메시지를 사용한다.

영상 옆 **MP4 저장**을 누르면 선택한 기록의 전체 HUD 영상을 다운로드한다.
기록된 프레임 간격을 유지하며, 뷰어 실행 컴퓨터에 FFmpeg(libx264)가 필요하다.
macOS에서는 `brew install ffmpeg`, Ubuntu에서는 `sudo apt install ffmpeg`로 설치할 수 있다.

`Video + Graphs`의 영상 아래 3D 뷰에서 드론 궤적, 드론이 추정한 마커, 시뮬레이터의 실제 마커를 같은 시간축으로 볼 수 있다.
마우스 드래그로 회전하고 휠로 확대한다. **보기 중심**의 **원점**은 ARM 위치를 (0, 0, 0)으로 표시하고,
**드론 따라가기**는 드론 위치를 따라간다. 영상이 없는 기록도 3D 재생과 탐색이 가능하다.
ARM 전환 이후부터 녹화된 로그에서는 첫 유효 드론 위치를 원점으로 쓴다.

SITL 및 HITL에서 Gazebo가 제공하는 마커와 드론의 실제 위치를 bag에 기록한다. 뷰어는 첫 공통
드론 위치 샘플로 Gazebo ENU 원점을 PX4 로컬 원점에 맞춘다. 실제 마커 위치가 기록된 bag에만
실제 마커가 표시된다.

`Marker relative to drone` 그래프는 기체 기준 FRD 좌표에서 ArUco 측정값과
`/pl/control/status`에 기록된 KF 값을 비교한다. KF 점선은 컨트롤러가 수평
속도 명령에 사용한 최신 타겟 추정값을 같은 제어 주기의 기체 자세로 변환해
기록한 값이다. 이 필드가 없는 기존 bag에서는 ArUco 측정값만 표시된다.

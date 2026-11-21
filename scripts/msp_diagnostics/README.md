# Jetson–Betaflight MSP 계측

`probe`는 UART를 단독으로 열고 읽기 요청을 보낸다. `live`는 `ros2 launch`나 `ros2 run`을 실행하면서 현재 설치된 ADR MSP 클라이언트의 TX, RX, 재연결과 ROS 발행을 기록한다. `analyze`는 저장한 데이터를 맥이나 Jetson에서 다시 분석한다.

## 준비

Jetson에서 저장소 루트를 열고 기존 launch를 종료한다. 프롭은 빼고 DISARM·OVERRIDE OFF 상태에서 시작한다. Betaflight 앱에서 현재 MSP UART의 baud와 CLI `get serial_update_rate_hz` 출력을 확인한 뒤, CLI를 종료하고 앱에서 Disconnect 한다. 4.4.3에서는 CLI가 열려 있으면 표준 MSP 응답 처리가 멈춘다. [Betaflight 4.4.3 시리얼 태스크](https://github.com/betaflight/betaflight/blob/4.4.3/src/main/fc/tasks.c#L130-L146)

`probe`에는 Python 3.10 이상과 `python3-serial`이 필요하다. `live`에는 Jetson에서 평소 쓰는 ROS·워크스페이스 소싱과 카메라·송출 환경도 필요하다. `analyze`는 Python 표준 라이브러리만 사용한다.

```bash
diag=scripts/msp_diagnostics/msp_diagnostics.py
baud=500000  # FC Ports에서 Jetson과 연결된 UART에 저장된 값
fc_serial_hz=100  # FC에서 실제로 읽은 serial_update_rate_hz
logs=flight_logs/msp_diagnostics/$(date +%Y%m%d-%H%M%S)
```

아래 명령은 한 번에 하나씩 실행한다. `--out`은 실행마다 새 디렉터리를 지정한다. 생략하면 `flight_logs/msp_diagnostics/` 아래에 타임스탬프를 포함한 새 디렉터리가 생긴다.

## 1. UART 속도와 부하 비교

먼저 5 Hz `FC_VARIANT`를 현재 baud에서 실행한다. 115200과 500000을 비교할 때는 FC 설정을 저장하고 양쪽 baud를 함께 바꾼 상태에서 같은 테스트를 반복한다.

FC가 이미 500000 baud라면 아래 한 줄로 `/dev/ttyTHS1`에서 3분간 기준 테스트를 실행한다.

```bash
./scripts/msp_diagnostics/probe_500000.sh
```

baud를 바꿔 비교하거나 경로와 FC 시리얼 주기까지 함께 지정하려면 Python 명령을 사용한다.

```bash
python3 "$diag" probe --baud "$baud" --fc-serial-hz "$fc_serial_hz" \
  --profile variant --hz 5 --seconds 180 --out "$logs/variant-$baud"
```

저부하에서 연결이 잘 되면 같은 baud에서 아래 순서로 비교한다. 타임아웃은 기본 25 ms다. `full`은 IMU·자세에 RC 25 Hz, FC 상태 10 Hz, 배터리 2 Hz를 추가하고 RC를 읽는 요청에는 상태도 함께 넣는다.

```bash
python3 "$diag" probe --baud "$baud" --fc-serial-hz "$fc_serial_hz" \
  --profile imu --hz 25 --out "$logs/imu25-$baud"
python3 "$diag" probe --baud "$baud" --fc-serial-hz "$fc_serial_hz" \
  --profile imu --hz 100 --out "$logs/imu100-$baud"
python3 "$diag" probe --baud "$baud" --fc-serial-hz "$fc_serial_hz" \
  --profile full --hz 100 --out "$logs/full100-$baud"
```

끊긴 프로필의 baud와 주기를 유지한 채 `--timeout-ms 100 --deadline-ms 25`를 추가해 반복한다. 25 ms를 넘긴 유효 프레임은 `late`로 집계된다. 배치 자체를 비교하려면 같은 주기에서 `--individual`을 추가한다. 이때 같은 조회들을 각각 별도 요청으로 보내므로 FC가 처리하는 요청 수가 늘어난다.

```bash
python3 "$diag" probe --baud "$baud" --fc-serial-hz "$fc_serial_hz" \
  --profile full --hz 100 --timeout-ms 100 --deadline-ms 25 \
  --out "$logs/full100-long-timeout-$baud"
```

위 비교에서 실패한 프로필이 `variant`나 `imu`였다면 해당 `--profile`과 `--hz`를 그대로 쓴다. 100 ms는 늦은 응답을 관찰하기 위한 설정이다.

## 2. 실제 런치에서 재현

Jetson에서 평소 쓰는 ROS·워크스페이스 환경을 소싱한 터미널에서 실행한다. `live`는 `msp_telemetry` 프로세스 안에서 실제 시리얼 호출을 계측한다. `Ctrl-C`로 종료하면 자동으로 요약을 만든다.

```bash
python3 "$diag" live --fc-serial-hz "$fc_serial_hz" --out "$logs/real-rx-$baud" -- \
  ros2 launch adr_bringup real.launch.py msp_baud:="$baud" policy_rc:=false
```

조회가 안정되면 프롭을 뺀 상태에서 `policy_rc:=true`로 실행해 ARM·ANGLE·OVERRIDE 인계 구간을 별도 기록한다. 이 모드에서는 기존 런치가 실제 RC를 송신한다. 4.4.3은 UART가 끊겨도 마지막 MSP 스틱값을 유지할 수 있으므로, 테스트 중 조종기의 OVERRIDE OFF와 DISARM을 사용할 수 있게 둔다. [4.4.3 OVERRIDE 구현](https://github.com/betaflight/betaflight/blob/4.4.3/src/main/rx/msp_override.c#L27-L38)

```bash
python3 "$diag" live --fc-serial-hz "$fc_serial_hz" --out "$logs/real-auto-$baud" -- \
  ros2 launch adr_bringup real.launch.py msp_baud:="$baud" policy_rc:=true
```

FC 시리얼 태스크의 영향을 보려면 같은 baud와 런치 설정에서 `serial_update_rate_hz`만 바꾸고, 바꾼 값을 `--fc-serial-hz`로 표시해 다른 디렉터리에 반복 기록한다. Betaflight 4.4.3은 한 UART에서 태스크 실행 한 번에 MSP 명령 하나를 처리한다. [4.4.3 MSP 처리 루프](https://github.com/betaflight/betaflight/blob/4.4.3/src/main/msp/msp_serial.c#L487-L501)

기준 로그를 확보한 뒤 100 Hz에서 250 Hz로 바꾸는 비교라면, 계측을 종료하고 Betaflight CLI에서 아래처럼 적용한다. 앱을 Disconnect 한 뒤 `fc_serial_hz=250`으로 바꿔 다음 로그를 찍는다.

```text
get serial_update_rate_hz
set serial_update_rate_hz = 250
save
```

## 결과 확인과 업로드

| 파일 | 내용 |
|---|---|
| `run.json` | 실행 모드와 기록한 FC 설정 값 |
| `events-*.jsonl` | 원본 TX·RX hex, 읽기 시각, 단일 요청·배치 구성, 포트 열기·닫기, 예외와 ROS 상태 |
| `requests.csv` | 요청별 수신 바이트, 첫 바이트·유효 프레임 시간, 전송 시간 추정, 실패 분류 |
| `summary.json` | 요청 종류별 응답 시간 p50·p95·p99·최대값, 재오픈, FC 상태 전환, 타이머 및 IMU 발행률 |

- `no_rx_bytes`는 타임아웃까지 프로세스가 읽은 바이트가 0이라는 뜻이다.
- `checksum_error`, `incomplete_frame`, `no_matching_frame`은 읽은 바이트에서 발견한 문제다. `valid_frame_not_accepted`는 원본 바이트에는 일치하는 프레임이 있었지만 기존 클라이언트가 응답으로 반환하지 못한 경우다.
- `batch_errors`는 외부 MSP 프레임 안의 하위 응답 누락이나 길이 오류다. `SET_RAW_RC`는 조회와 별도로 집계된다.
- `first_rx_ms`와 `frame_ms`는 요청 시작부터 Jetson 프로세스가 해당 바이트를 읽은 시점까지다. `wire_ms`는 TX·RX 바이트 수로 계산한 8N1 전송 시간이다. FC 처리 시각을 직접 얻으려면 FC 쪽 계측이 필요하다. `live` 계측 자체에도 약간의 실행 비용이 있다.
- `dropped_events`, `incomplete_captures`와 `invalid_json_lines`는 캡처 결과가 온전히 저장됐는지 확인하는 항목이다.

저장한 로그를 다시 요약하거나, 여러 비교 결과를 한 번에 압축해 올릴 수 있다.

```bash
python3 "$diag" analyze "$logs/full100-long-timeout-$baud" --deadline-ms 25
tar -czf /tmp/adr-msp-diagnostics.tar.gz -C "$(dirname "$logs")" "$(basename "$logs")"
```

`adr-msp-diagnostics.tar.gz`와 증상이 나온 시점에 ARM·OVERRIDE가 켜져 있었는지 알려주면 된다. `live`의 rosbag은 별도 `flight_logs/`에 저장되므로 HUD에서만 연결이 깜빡인 경우 해당 rosbag도 같이 보내면 된다.

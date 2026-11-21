# Live HUD streaming

`adr_streaming` subscribes to the Jetson's local BGR HUD (`/adr/monitor/image`),
encodes H.264 with GStreamer and publishes to MediaMTX over RTSP. The real profile uses a 320×240 HUD with a 30 Hz render target.
The JPEG HUD in the bag plays back independently of the live stream.

## Jetson

Install the GStreamer plugins and introspection bindings, then build the real profile.

```bash
sudo apt install python3-gi gir1.2-gstreamer-1.0 gstreamer1.0-tools \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-rtsp
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Install MediaMTX on the Jetson. The real launch looks for `mediamtx` on
`PATH` or under `$ADR_ROOT/.deps/hud/mediamtx`. From the repository root:

```bash
ros2 launch adr_bringup real.launch.py policy_rc:=false
```

The launch starts MediaMTX with the packaged configuration and publishes
HUD video to it at `rtsp://127.0.0.1:8554/hud`. Both processes stop with
the launch. To disable streaming for a run, pass `stream_url:=`.
If a MediaMTX server is already running, supply its RTSP publish URL:

```bash
ros2 launch adr_bringup real.launch.py policy_rc:=false stream_url:=rtsp://127.0.0.1:8554/hud
```

On the ground station, open `http://JETSON_LAN_IP:8889/hud` in a browser
for WebRTC playback, or open `rtsp://JETSON_LAN_IP:8554/hud` in an RTSP player.
With the default MediaMTX ports, browser playback needs TCP 8889 and UDP 8189
reachable on the Jetson. For RTSP playback over TCP, allow TCP 8554.
The Jetson publisher uses RTSP over UDP on loopback by default.

The defaults are 320×240, up to 25 fps, 450 kbps, x264 ultrafast/zerolatency,
no B-frames, an IDR at most every 0.5 s at full frame rate, and one queued
raw frame in GStreamer. Keep the raw HUD subscriber on Jetson to avoid
sending the raw feed over DDS. The stream can also be started separately to
adjust bitrate, frame rate, size or try TCP when UDP is unavailable. For
this setup, start MediaMTX separately and pass `stream_url:=` to the real launch.

```bash
source install/setup.bash
export ROS_DOMAIN_ID=0  # Match real.yaml if customized.
ros2 run adr_streaming hud_streamer --ros-args \
  -p url:=rtsp://127.0.0.1:8554/hud -p rtsp_transport:=tcp
```

## Low-latency playback with GStreamer

On the ground-station Mac, run this command with your Jetson's LAN IP.

```bash
gst-launch-1.0 -v \
  rtspsrc location=rtsp://10.164.13.71:8554/hud \
    protocols=udp latency=20 drop-on-latency=true do-retransmission=false \
  ! rtph264depay ! h264parse ! avdec_h264 \
  ! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream \
  ! videoconvert ! autovideosink sync=false
```

If the video breaks up on Wi-Fi, increase `latency` to `50` ms. If UDP
playback receives no video, switch to `protocols=tcp`.

Start by comparing the video delay and GateNet throughput with live
streaming off and on. Verify the recorded JPEG frame intervals in a fresh
flight bag. Existing bags retain their original frame rate.

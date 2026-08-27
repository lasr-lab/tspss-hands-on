# UDP Decoder (receiving side)

Counterpart to `image_udp_bridge/udp_sender.py`. It reassembles the chunked
JSON stream the ROS2 node sends and turns it back into usable data — no ROS
installation needed on this side.

| Stream | What the decoder does |
| --- | --- |
| `color_image` | live OpenCV window per sensor, optional `.mp4` / JPEG recording |
| `audio` | live playback of one mic channel, optional per-channel `.wav` |
| `pressure`, `pressure_ap`, `imu_raw`, `imu_euler`, `imu_quat`, `gas` | one `<type>.jsonl` per stream + a live status line |

## Install

```bash
pip install -r requirements.txt          # numpy is required, the rest optional
sudo apt install libportaudio2           # only for --play
```

Without `opencv-python` there are no live windows, but everything else still
works — including `--save-frames`, which writes the incoming JPEGs straight to
disk without decoding them. Without `sounddevice` everything but `--play` works.

## No video?

The stats line ends with `frames=`, so check it first:

- `frames=0 (no images received)` — nothing is arriving. Either the sender has
  `enable_images:=false`, or nothing is publishing
  `/image_raw/compressed/index_<i>` (that comes from `d360_image_pub`, so a
  sensor-only launch produces no images), or the images are being lost on the
  link. A rising `incomplete=` in the same line means the latter: images are
  chunked into ~40 KB UDP packets and a single missing chunk discards the whole
  frame. Cap the chatty sensor streams to free up bandwidth:
  `-p sensor_max_hz:=50` on the sender, or turn `enable_pressure_ap:=false` off
  while debugging — it alone is a few hundred KB/s.
- `frames=` counting up but no window — OpenCV is missing (see the notice at
  startup) or you passed `--no-video`. `--save-frames` gets you the images
  regardless.

## Run

```bash
# live video + live audio, nothing written to disk
./udp_decoder.py --play

# same, but also record video and audio into ./capture
./udp_decoder.py --play --record-video --record-audio

# headless logging only
./udp_decoder.py --no-video --out-dir ~/runs/2026-08-21
```

Defaults are `--bind 192.168.1.29 --port 8080`, matching the sender's default
`target_ip` / `target_port`. Binding to that address only works if it really is
an address of this machine — use `--bind 0.0.0.0` to listen on every interface.
The sender's `target_ip` must point at this machine:

```bash
ros2 run image_udp_bridge udp_sender --ros-args \
  -p target_ip:=192.168.1.29 -p target_port:=8080 -p device_count:=1
```

In a video window: `q` / `ESC` quits, `s` toggles JPEG frame saving.

## Options worth knowing

- `--play-channel N` — which mic to play (default `0`). Channels arrive as
  separate ROS topics, so playback uses one mono stream per sensor rather than
  interleaving them, which would drift the moment a packet is lost.
- `--audio-buffer-ms` — playback backlog cap (default `120`). Lower means less
  latency and more dropouts.
- `--video-fps` — frame rate stamped into `--record-video` output. The stream
  itself is not constant-rate, so set this to the sensor's actual rate if the
  recording should play back at real speed.
- `--chunk-timeout` — how long a partially received image is kept before it is
  discarded (default `2.0` s).
- `--report-interval` — seconds between `[stats]` lines.

## Reading the stats line

```
[stats] audio=10.0/s 15KB/s chunk=20.0/s 944KB/s color_image=10.0/s gas=10.0/s ...
        gas[0] 12345 41.2%RH 24.5C | imu[0]/ACC +1.90,+0.00,+9.80
```

Chunked messages (images, mostly) have their bytes booked under `chunk`, while
the reassembled message is counted under its own type — so `color_image` shows
a rate but no byte figure. `incomplete=` counts images dropped because not
every chunk arrived, which is the number to watch when the link saturates.

## Analysing the logs afterwards

Each `.jsonl` line is one ROS message, with the sender's fields unchanged:

```python
import json
with open("capture/imu_raw.jsonl") as f:
    samples = [json.loads(line) for line in f]
acc = [s for s in samples if s["sensor_type"] == 1]   # 1:ACC 2:GYRO 3:MAG 6:LINACC
```

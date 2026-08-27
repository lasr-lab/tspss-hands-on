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
`target_ip` / `target_port`. The sender's `target_ip` must point at this machine:

```bash
ros2 run image_udp_bridge udp_sender --ros-args \
  -p target_ip:=192.168.1.29 -p target_port:=8080 -p device_count:=1
```

**`--bind` takes an address of the machine the decoder runs on, never the
sender's.** Passing a foreign address fails with `Cannot assign requested
address` (errno 99). `--bind 0.0.0.0` listens on every interface and always
works — use it when in doubt.

In a video window: `q` / `ESC` quits, `s` toggles JPEG frame saving.

## Several receivers at once (multicast)

When the sender streams to a multicast group (`target_ip` in `224.0.0.0/4`),
any number of machines can receive it and the sender needs none of their
addresses. Each receiver joins the group:

```bash
# preferred form - works on every OS
./udp_decoder.py --bind 0.0.0.0 --multicast-group 239.255.42.1 --port 8081

# shorthand: binding the group address joins it (Linux/macOS only)
./udp_decoder.py --bind 239.255.42.1 --port 8081
```

On success it prints `[udp] joined multicast group 239.255.42.1`. Windows
refuses to bind a socket to a multicast address, so the shorthand fails there;
the decoder falls back to `0.0.0.0` automatically, but `--multicast-group` is
the form to use.

On a machine with several interfaces (WiFi + Ethernet + docker0), pin the one
facing the sender with **its own local IP** — this is the interface the group is
joined on, not the sender's address:

```bash
./udp_decoder.py --bind 0.0.0.0 --multicast-group 239.255.42.1 --iface 192.168.200.42
```

The sender has the matching `-p multicast_iface:=<its own IP>`.

Multicast is a poor fit for busy WiFi: access points send it at the lowest basic
rate with no link-layer retries, and many block it between clients entirely.
Expect more dropped frames than with unicast, and see below if nothing arrives.

## Dropped frames

Images are chunked into ~1400-byte packets and every chunk must arrive, so on a
lossy link the frame rate collapses long before the link is saturated. Two
things in the protocol deal with that, both automatic:

- images travel as raw binary (`b"D360"` + 30-byte header + JPEG) instead of
  base64 inside JSON, which is a quarter fewer bytes;
- each frame carries one XOR parity packet, so any single lost chunk is
  reconstructed instead of costing the frame. Measured delivery at 2% packet
  loss: 72% without, 94% with.

The stats line reports repairs as they happen:

```
[stats] ... | incomplete=3 malformed=0 | fec_repaired=112 | frames=940
```

`fec_repaired` climbing means the link is lossy and the parity is earning its
keep. `incomplete` climbing means frames are losing *more than one* chunk —
reduce the bandwidth (`max_image_size`, `image_max_hz`) rather than expecting
FEC to cover it.

**This decoder must match the sender.** An older decoder cannot read the binary
images and will count them as `malformed` while `frames=0`; start the sender
with `-p binary_images:=false` if some machine cannot be updated.

## Nothing arrives at all

Beyond `frames=0`, when no stream shows up in the stats line:

1. **Check the wire first.** `sudo tcpdump -ni any port 8081` on this machine.
   Nothing there means the packets never made it, and no decoder flag will help.
2. **Same subnet?** With the sender's default `multicast_ttl:=1` the traffic
   does not cross a router. Both machines must be on the same subnet — compare
   `ip -4 addr` on each.
3. **Access point blocking.** Client isolation and multicast filtering are
   common on guest and conference WiFi. Test with unicast: point the sender's
   `target_ip` at this machine's IP and run `./udp_decoder.py --bind 0.0.0.0`.
   If unicast works and multicast does not, it is the AP — use the sender's
   `target_ips:="[ip1, ip2]"` to fan out to the machines you need instead.
4. **Firewall.** `sudo ufw status` — UDP on the chosen port must be allowed.

## Options worth knowing

- `--multicast-group ADDR` — join a multicast group, independent of `--bind`.
- `--iface IP` — local IP of the interface to join the group on (default
  `0.0.0.0`, i.e. let the kernel choose by route).
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

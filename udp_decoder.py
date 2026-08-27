#!/usr/bin/env python3
"""
UDP Decoder: receiving side for image_udp_bridge/udp_sender.py.

Reassembles the chunked JSON stream sent by the ROS2 node and turns it back
into usable data:

  color_image  -> live OpenCV window per DIGIT360 sensor (optional .mp4/.jpg recording)
  audio        -> live playback (sounddevice) and/or per-channel .wav files
  pressure / pressure_ap / imu_* / gas
               -> newline-delimited JSON logs, one file per stream type

Nothing here needs ROS. Only numpy is required; OpenCV (windows/recording) and
sounddevice (playback) are optional and degrade gracefully when missing.

Wire format (see udp_sender.py):
  {"type": "<stream>", "data": {...}}                         plain message
  {"type": "chunk", "msg_type": ..., "message_id": ...,       fragment of a
   "chunk_index": i, "total_chunks": n, "data": "<b64>"}      large message

Usage:
  ./udp_decoder.py --port 8081
  ./udp_decoder.py --port 8081 --play --record-audio --record-video
  ./udp_decoder.py --port 8081 --no-video --out-dir ./capture

To receive a multicast stream (sender running with a 224.0.0.0/4 target_ip),
pass the group as --bind; any number of machines can do this at the same time
and the sender needs none of their addresses:
  ./udp_decoder.py --bind 239.255.42.1 --port 8081
"""

import argparse
import base64
import binascii
import collections
import json
import os
import signal
import socket
import sys
import threading
import time
import wave

import numpy as np

# Optional dependencies -------------------------------------------------------
try:
    import cv2
except ImportError:  # pragma: no cover - environment dependent
    cv2 = None

try:
    import sounddevice as sd
except Exception:  # pragma: no cover - import can fail on missing PortAudio
    sd = None


# Matches the sender's defaults (target_ip / target_port in udp_sender.py).
DEFAULT_BIND = "192.168.1.29"
DEFAULT_PORT = 8080

SENSOR_TYPES = ("pressure", "pressure_ap", "imu_raw", "imu_euler", "imu_quat", "gas")
IMU_SENSOR_NAMES = {1: "ACC", 2: "GYRO", 3: "MAG", 6: "LINACC"}


def is_multicast(ip: str) -> bool:
    """True for the 224.0.0.0/4 range."""
    try:
        return 224 <= int(ip.split(".", 1)[0]) <= 239
    except ValueError:
        return False


# ---------------------------------------------------------------------------
# Chunk reassembly
# ---------------------------------------------------------------------------


class ChunkReassembler:
    """Collects `chunk` fragments until a message is complete.

    UDP gives no ordering or delivery guarantees, so partial messages are
    dropped once they go stale instead of being kept forever.
    """

    def __init__(self, timeout=2.0):
        self.timeout = timeout
        self._pending = {}
        self.completed = 0
        self.expired = 0

    def add(self, chunk: dict):
        message_id = chunk["message_id"]
        total = chunk["total_chunks"]
        entry = self._pending.get(message_id)
        if entry is None or entry["total"] != total:
            entry = {"total": total, "parts": {}, "t": time.monotonic()}
            self._pending[message_id] = entry

        try:
            entry["parts"][chunk["chunk_index"]] = base64.b64decode(chunk["data"])
        except (binascii.Error, ValueError):
            return None

        if len(entry["parts"]) < total:
            return None

        del self._pending[message_id]
        payload = b"".join(entry["parts"][i] for i in range(total))
        try:
            message = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return None
        self.completed += 1
        return message

    def collect_garbage(self):
        deadline = time.monotonic() - self.timeout
        stale = [mid for mid, e in self._pending.items() if e["t"] < deadline]
        for mid in stale:
            del self._pending[mid]
        self.expired += len(stale)


# ---------------------------------------------------------------------------
# Video
# ---------------------------------------------------------------------------


class VideoSink:
    """Holds the newest JPEG per sensor index; decoding happens on the UI thread.

    Only the latest frame is kept: on a congested link, showing a backlog of
    stale frames is worse than dropping them.
    """

    def __init__(self, out_dir, record=False, save_frames=False, fourcc="mp4v", fps=30.0):
        self.out_dir = out_dir
        self.record = record
        self.save_frames = save_frames
        self.fourcc = fourcc
        self.fps = fps
        self._latest = {}
        self._lock = threading.Lock()
        self._writers = {}
        self._frame_dirs = set()
        self.received = collections.Counter()

    def submit(self, index: int, data: dict):
        try:
            jpeg = base64.b64decode(data["data"])
        except (binascii.Error, ValueError, KeyError):
            return
        self.received[index] += 1
        if self.save_frames:
            # The payload is already JPEG, so this path needs no decoder at all
            # and works headless - handy for confirming frames actually arrive.
            self._save_jpeg(index, jpeg)
        with self._lock:
            self._latest[index] = (jpeg, data)

    def _save_jpeg(self, index: int, jpeg: bytes):
        frame_dir = os.path.join(self.out_dir, f"frames_index_{index}")
        if index not in self._frame_dirs:
            os.makedirs(frame_dir, exist_ok=True)
            self._frame_dirs.add(index)
            print(f"[video] saving frames of sensor {index} -> {frame_dir}/")
        path = os.path.join(frame_dir, f"{self.received[index]:06d}.jpg")
        with open(path, "wb") as handle:
            handle.write(jpeg)

    def take_all(self):
        with self._lock:
            frames = self._latest
            self._latest = {}
        return frames

    def write(self, index: int, frame):
        """Append a decoded frame to the video file (OpenCV only)."""
        if not self.record:
            return
        writer = self._writers.get(index)
        if writer is None:
            path = os.path.join(self.out_dir, f"index_{index}.mp4")
            height, width = frame.shape[:2]
            writer = cv2.VideoWriter(
                path, cv2.VideoWriter_fourcc(*self.fourcc), self.fps, (width, height)
            )
            self._writers[index] = writer
            print(f"[video] recording sensor {index} -> {path} ({width}x{height} @ {self.fps} fps)")
        writer.write(frame)

    def close(self):
        for writer in self._writers.values():
            writer.release()
        self._writers.clear()


# ---------------------------------------------------------------------------
# Audio
# ---------------------------------------------------------------------------


class AudioSink:
    """Decodes pcm_s16le payloads, optionally playing and/or recording them.

    Playback uses one mono stream per sensor for a single selected mic channel;
    the channels arrive as independent ROS topics, so mixing them into one
    interleaved stream would drift as soon as a packet is lost.
    """

    def __init__(self, out_dir, play=False, play_channel=0, record=False, buffer_ms=120):
        self.out_dir = out_dir
        self.play = play and sd is not None
        self.play_channel = play_channel
        self.record = record
        self.buffer_ms = buffer_ms
        self._wavs = {}
        self._streams = {}
        self._buffers = {}
        self._lock = threading.Lock()
        self.samples = collections.Counter()
        self.underruns = 0

        if play and sd is None:
            print("[audio] playback requested but sounddevice is unavailable - skipping", file=sys.stderr)

    def submit(self, index: int, data: dict):
        if data.get("format") != "pcm_s16le":
            return
        try:
            raw = base64.b64decode(data["data"])
        except (binascii.Error, ValueError, KeyError):
            return
        channel = int(data.get("channel", 0))
        rate = int(data.get("sample_rate", 48000)) or 48000
        # The sender byteswaps to little-endian regardless of host endianness.
        pcm = np.frombuffer(raw[: len(raw) - len(raw) % 2], dtype="<i2")
        if pcm.size == 0:
            return
        self.samples[(index, channel)] += pcm.size

        if self.record:
            self._write_wav(index, channel, rate, pcm)
        if self.play and channel == self.play_channel:
            self._enqueue(index, rate, pcm)

    def _write_wav(self, index, channel, rate, pcm):
        key = (index, channel)
        wav = self._wavs.get(key)
        if wav is None:
            os.makedirs(self.out_dir, exist_ok=True)
            path = os.path.join(self.out_dir, f"audio_index_{index}_mic_{channel}.wav")
            wav = wave.open(path, "wb")
            wav.setnchannels(1)
            wav.setsampwidth(2)
            wav.setframerate(rate)
            self._wavs[key] = wav
            print(f"[audio] recording sensor {index} mic {channel} -> {path} ({rate} Hz)")
        wav.writeframes(pcm.tobytes())

    def _enqueue(self, index, rate, pcm):
        with self._lock:
            buf = self._buffers.get(index)
            if buf is None:
                buf = collections.deque()
                self._buffers[index] = buf
                self._open_stream(index, rate)
            buf.append(pcm.astype(np.float32) / 32767.0)
            # Bound the backlog so a burst does not turn into growing latency.
            max_chunks = max(2, int(self.buffer_ms * rate / 1000 / max(1, pcm.size)))
            while len(buf) > max_chunks:
                buf.popleft()

    def _open_stream(self, index, rate):
        def callback(outdata, frames, time_info, status):
            del time_info
            if status:
                self.underruns += 1
            filled = 0
            with self._lock:
                buf = self._buffers.get(index) or collections.deque()
                while filled < frames and buf:
                    chunk = buf[0]
                    take = min(frames - filled, chunk.size)
                    outdata[filled : filled + take, 0] = chunk[:take]
                    if take == chunk.size:
                        buf.popleft()
                    else:
                        buf[0] = chunk[take:]
                    filled += take
            if filled < frames:
                outdata[filled:, 0] = 0.0  # starved: play silence, never block

        try:
            stream = sd.OutputStream(
                samplerate=rate, channels=1, dtype="float32", callback=callback,
                blocksize=0, latency="low",
            )
            stream.start()
            self._streams[index] = stream
            print(f"[audio] playing sensor {index} mic {self.play_channel} @ {rate} Hz")
        except Exception as exc:  # pragma: no cover - depends on audio hardware
            print(f"[audio] could not open output stream: {exc}", file=sys.stderr)
            self.play = False

    def close(self):
        for stream in self._streams.values():
            try:
                stream.stop()
                stream.close()
            except Exception:
                pass
        self._streams.clear()
        for wav in self._wavs.values():
            wav.close()
        self._wavs.clear()


# ---------------------------------------------------------------------------
# Scalar sensor streams
# ---------------------------------------------------------------------------


class SensorSink:
    """Appends every scalar/array sample to a per-type JSONL file and keeps the
    latest value of each stream around for the status line."""

    def __init__(self, out_dir, record=True):
        self.out_dir = out_dir
        self.record = record
        self._files = {}
        self.latest = {}

    def submit(self, wire_type: str, data: dict):
        self.latest[(wire_type, data.get("index", 0))] = data
        if not self.record:
            return
        handle = self._files.get(wire_type)
        if handle is None:
            os.makedirs(self.out_dir, exist_ok=True)
            path = os.path.join(self.out_dir, f"{wire_type}.jsonl")
            handle = open(path, "a", buffering=1 << 16)
            self._files[wire_type] = handle
            print(f"[sensor] logging {wire_type} -> {path}")
        handle.write(json.dumps(data, separators=(",", ":")) + "\n")

    def summary(self) -> str:
        parts = []
        for (wire_type, index), data in sorted(self.latest.items()):
            if wire_type == "imu_raw":
                name = IMU_SENSOR_NAMES.get(data.get("sensor_type"), "?")
                parts.append(
                    f"imu[{index}]/{name} "
                    f"{data.get('x', 0):+.2f},{data.get('y', 0):+.2f},{data.get('z', 0):+.2f}"
                )
            elif wire_type == "imu_euler":
                parts.append(
                    f"euler[{index}] h{data.get('heading', 0):+.1f} "
                    f"p{data.get('pitch', 0):+.1f} r{data.get('roll', 0):+.1f}"
                )
            elif wire_type == "imu_quat":
                parts.append(
                    f"quat[{index}] {data.get('x', 0):+.2f},{data.get('y', 0):+.2f},"
                    f"{data.get('z', 0):+.2f},{data.get('w', 0):+.2f}"
                )
            elif wire_type == "pressure":
                parts.append(
                    f"press[{index}] {data.get('pressure', 0):.0f}Pa "
                    f"{data.get('temperature', 0):.1f}C"
                )
            elif wire_type == "pressure_ap":
                ch_a = data.get("ch_a") or [0]
                parts.append(f"press_ap[{index}] n={len(ch_a)} a0={ch_a[0]}")
            elif wire_type == "gas":
                parts.append(
                    f"gas[{index}] {data.get('gas', 0):.0f} "
                    f"{data.get('humidity', 0):.1f}%RH {data.get('temperature', 0):.1f}C"
                )
        return " | ".join(parts)

    def close(self):
        for handle in self._files.values():
            handle.close()
        self._files.clear()


# ---------------------------------------------------------------------------
# Receiver
# ---------------------------------------------------------------------------


class RateCounter:
    """Counts messages and bytes per stream over a sliding report interval."""

    def __init__(self):
        self.messages = collections.Counter()
        self.bytes = collections.Counter()
        self.t0 = time.monotonic()

    def add(self, wire_type, nbytes):
        self.messages[wire_type] += 1
        self.bytes[wire_type] += nbytes

    def add_reassembled(self, wire_type):
        """Count a message whose bytes were already booked under `chunk`."""
        self.messages[wire_type] += 1

    def report_and_reset(self) -> str:
        elapsed = max(1e-6, time.monotonic() - self.t0)
        parts = [
            f"{name}={self.messages[name] / elapsed:.1f}/s "
            f"{self.bytes[name] / elapsed / 1024:.0f}KB/s"
            for name in sorted(self.messages)
        ]
        self.messages.clear()
        self.bytes.clear()
        self.t0 = time.monotonic()
        return " ".join(parts) if parts else "no data"


class UDPDecoder:
    def __init__(self, args):
        self.args = args
        os.makedirs(args.out_dir, exist_ok=True)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, "SO_REUSEPORT"):
            # Lets several decoders on one machine share the port, which is the
            # normal case for multicast.
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        # A generous receive buffer: the sender bursts chunked JPEG frames.
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, args.rcvbuf)

        # The group to join and the local address to bind are independent:
        # --bind <group> is a shorthand, --multicast-group works with any bind
        # address (and is the only form Windows accepts, since it refuses to
        # bind a socket to a multicast address).
        self.multicast_group = args.multicast_group or (
            args.bind if is_multicast(args.bind) else None
        )
        bind_addr = args.bind
        try:
            self.sock.bind((bind_addr, args.port))
        except OSError as exc:
            if self.multicast_group and is_multicast(bind_addr):
                print(f"[udp] cannot bind the group address directly ({exc}), "
                      f"binding 0.0.0.0 instead", file=sys.stderr)
                bind_addr = "0.0.0.0"
                self.sock.bind((bind_addr, args.port))
            else:
                raise SystemExit(
                    f"[udp] cannot bind {bind_addr}:{args.port} - {exc}\n"
                    f"      --bind takes an address of THIS machine (not the "
                    f"sender's) - use --bind 0.0.0.0 to listen on every "
                    f"interface, and --multicast-group <addr> to join a group."
                ) from exc

        if self.multicast_group:
            # Joining tells the switch/AP to forward the group to this host; the
            # sender never learns that we exist.
            mreq = socket.inet_aton(self.multicast_group) + socket.inet_aton(args.iface)
            try:
                self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
            except OSError as exc:
                raise SystemExit(
                    f"[udp] cannot join multicast group {self.multicast_group} - {exc}\n"
                    f"      pass --iface <local ip of the interface facing the sender>."
                ) from exc
            print(f"[udp] joined multicast group {self.multicast_group}"
                  + (f" on {args.iface}" if args.iface != "0.0.0.0" else ""))

        self.sock.settimeout(0.5)

        self.reassembler = ChunkReassembler(timeout=args.chunk_timeout)
        self.rates = RateCounter()
        self.video = VideoSink(
            args.out_dir, record=args.record_video, save_frames=args.save_frames,
            fourcc=args.fourcc, fps=args.video_fps,
        )
        self.audio = AudioSink(
            args.out_dir, play=args.play, play_channel=args.play_channel,
            record=args.record_audio, buffer_ms=args.audio_buffer_ms,
        )
        self.sensors = SensorSink(args.out_dir, record=args.log_sensors)

        self.unknown_types = collections.Counter()
        self.malformed = 0
        self._stop = threading.Event()
        self._frame_times = collections.defaultdict(collections.deque)

    # -- network thread ---------------------------------------------------

    def _receive_loop(self):
        while not self._stop.is_set():
            try:
                packet, _addr = self.sock.recvfrom(65535)
            except socket.timeout:
                self.reassembler.collect_garbage()
                continue
            except OSError:
                break

            try:
                message = json.loads(packet.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                self.malformed += 1
                continue

            wire_type = message.get("type")
            if wire_type == "chunk":
                self.rates.add("chunk", len(packet))
                message = self.reassembler.add(message)
                if message is None:
                    continue
                wire_type = message.get("type")
                self.rates.add_reassembled(wire_type or "?")
            else:
                self.rates.add(wire_type or "?", len(packet))

            self._dispatch(wire_type, message.get("data") or {})

    def _dispatch(self, wire_type, data):
        index = int(data.get("index", 0))
        if wire_type == "color_image":
            self.video.submit(index, data)
        elif wire_type == "audio":
            self.audio.submit(index, data)
        elif wire_type in SENSOR_TYPES:
            self.sensors.submit(wire_type, data)
        else:
            self.unknown_types[wire_type] += 1

    # -- main thread ------------------------------------------------------

    def _fps(self, index) -> float:
        times = self._frame_times[index]
        now = time.monotonic()
        times.append(now)
        while times and now - times[0] > 2.0:
            times.popleft()
        return (len(times) - 1) / max(1e-6, now - times[0]) if len(times) > 1 else 0.0

    def _show_frames(self) -> bool:
        """Decode and display pending frames. Returns False when the user quits."""
        for index, (jpeg, meta) in sorted(self.video.take_all().items()):
            frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                self.malformed += 1
                continue
            self.video.write(index, frame)
            fps = self._fps(index)
            label = f"index {index}  {frame.shape[1]}x{frame.shape[0]}  {fps:4.1f} fps"
            cv2.putText(frame, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0), 1, cv2.LINE_AA)
            cv2.imshow(f"DIGIT360 index_{index}", frame)
            del meta

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            return False
        if key == ord("s"):
            self.args.save_frames = self.video.save_frames = not self.video.save_frames
            print(f"[video] frame saving {'on' if self.video.save_frames else 'off'}")
        return True

    def run(self):
        # Close recordings cleanly when the process is asked to terminate.
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: self._stop.set())

        thread = threading.Thread(target=self._receive_loop, daemon=True)
        thread.start()

        show_video = self.args.video and cv2 is not None
        if self.args.video and cv2 is None:
            print("[video] OpenCV not installed - no live windows "
                  "(pip install opencv-python). --save-frames still writes the "
                  "incoming JPEGs, which is the way to check that frames arrive.",
                  file=sys.stderr)

        print(f"[udp] listening on {self.args.bind}:{self.args.port}, output in {self.args.out_dir}")
        if show_video:
            print("[udp] press q or ESC in a video window to quit, s to toggle frame saving")

        next_report = time.monotonic() + self.args.report_interval
        try:
            while not self._stop.is_set():
                if show_video:
                    if not self._show_frames():
                        break
                else:
                    time.sleep(0.05)

                now = time.monotonic()
                if now >= next_report:
                    next_report = now + self.args.report_interval
                    self._print_status()
        except KeyboardInterrupt:
            print()
        finally:
            self.close()

    def _print_status(self):
        line = f"[stats] {self.rates.report_and_reset()}"
        if self.reassembler.expired or self.malformed:
            line += f" | incomplete={self.reassembler.expired} malformed={self.malformed}"
        if self.audio.underruns:
            line += f" | audio_underruns={self.audio.underruns}"
        frames = sum(self.video.received.values())
        line += f" | frames={frames}" if frames else " | frames=0 (no images received)"
        if self.unknown_types:
            line += f" | unknown={dict(self.unknown_types)}"
        print(line)
        sensors = self.sensors.summary()
        if sensors:
            print(f"        {sensors}")

    def close(self):
        self._stop.set()
        if self.multicast_group:
            try:
                mreq = (socket.inet_aton(self.multicast_group)
                        + socket.inet_aton(self.args.iface))
                self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_DROP_MEMBERSHIP, mreq)
            except OSError:
                pass
        self.sock.close()
        self.video.close()
        self.audio.close()
        self.sensors.close()
        if cv2 is not None and self.args.video:
            cv2.destroyAllWindows()
        print("[udp] stopped")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Decode the DIGIT360 UDP stream produced by image_udp_bridge/udp_sender.py",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--bind", default=DEFAULT_BIND,
                        help="local interface to listen on (0.0.0.0 for all), or a "
                             "multicast group in 224.0.0.0/4 to join")
    parser.add_argument("--multicast-group", default=None,
                        help="multicast group to join, independent of --bind")
    parser.add_argument("--iface", default="0.0.0.0",
                        help="local IP of the interface to join the multicast group on")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT,
                        help="UDP port (sender's target_port)")
    parser.add_argument("--out-dir", default="./capture", help="directory for recordings and logs")
    parser.add_argument("--rcvbuf", type=int, default=8 << 20, help="SO_RCVBUF in bytes")
    parser.add_argument("--chunk-timeout", type=float, default=2.0,
                        help="seconds before an incomplete chunked message is dropped")
    parser.add_argument("--report-interval", type=float, default=5.0,
                        help="seconds between status lines")

    parser.add_argument("--no-video", dest="video", action="store_false",
                        help="do not open live video windows")
    parser.add_argument("--record-video", action="store_true", help="also write index_<i>.mp4")
    parser.add_argument("--save-frames", action="store_true", help="also write every frame as JPEG")
    parser.add_argument("--fourcc", default="mp4v", help="FourCC for --record-video")
    parser.add_argument("--video-fps", type=float, default=30.0,
                        help="frame rate stamped into the recorded video file")

    parser.add_argument("--play", action="store_true", help="play a mic channel live")
    parser.add_argument("--play-channel", type=int, default=0, help="mic channel to play")
    parser.add_argument("--audio-buffer-ms", type=int, default=120,
                        help="playback backlog cap; lower = less latency, more dropouts")
    parser.add_argument("--record-audio", action="store_true", help="write per-channel WAV files")

    parser.add_argument("--no-sensor-log", dest="log_sensors", action="store_false",
                        help="do not write the per-stream JSONL logs")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    UDPDecoder(args).run()
    return 0


if __name__ == "__main__":
    sys.exit(main())

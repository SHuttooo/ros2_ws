#!/usr/bin/env python3
"""
Serveur MJPEG HTTP - stream FFmpeg directement
"""

from http.server import HTTPServer, BaseHTTPRequestHandler
import subprocess
import sys

RTSP_URL = "rtsp://admin:ros2_2025@100.73.141.53:8554/h264Preview_01_main"
PORT = 8090
RTSP_TRANSPORT = "tcp"  # "udp" pour réduire la latence si le réseau le permet
MAX_BUFFER_BYTES = 1_000_000

ffmpeg_proc = None


class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        global ffmpeg_proc

        print(f"📡 Client connecté: {self.client_address[0]}", flush=True)

        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=BOUNDARY")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()

        try:
            buf = b""
            frame_count = 0

            while True:
                chunk = ffmpeg_proc.stdout.read(4096)
                if not chunk:
                    break

                buf += chunk

                if len(buf) > MAX_BUFFER_BYTES:
                    last_start = buf.rfind(b"\xff\xd8")
                    buf = buf[last_start:] if last_start != -1 else b""

                while b"\xff\xd8" in buf and b"\xff\xd9" in buf:
                    start = buf.find(b"\xff\xd8")
                    end = buf.find(b"\xff\xd9", start)

                    if end == -1:
                        break

                    frame = buf[start : end + 2]
                    frame_count += 1

                    response = (
                        b"--BOUNDARY\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        b"Content-Length: "
                        + str(len(frame)).encode()
                        + b"\r\n\r\n"
                        + frame
                        + b"\r\n"
                    )

                    self.wfile.write(response)

                    if frame_count % 10 == 0:
                        print(f"✅ Frame {frame_count} ({len(frame)} bytes)", flush=True)

                    buf = buf[end + 2 :]

        except Exception as e:
            print(f"❌ Erreur: {e}", flush=True)

    def log_message(self, format, *args):
        pass


ffmpeg_cmd = [
    "ffmpeg",
    "-fflags",
    "nobuffer",
    "-flags",
    "low_delay",
    "-max_delay",
    "0",
    "-rtsp_transport",
    RTSP_TRANSPORT,
    "-i",
    RTSP_URL,
    "-c:v",
    "mjpeg",
    "-q:v",
    "2",
    "-f",
    "mjpeg",
    "pipe:1",
]

print("🎬 Démarrage serveur MJPEG", flush=True)
print(f"Port: {PORT}", flush=True)

try:
    ffmpeg_proc = subprocess.Popen(
        ffmpeg_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        bufsize=0,
    )
    print("✅ FFmpeg OK", flush=True)
except Exception as e:
    print(f"❌ FFmpeg: {e}", flush=True)
    sys.exit(1)

print(f"✅ Serveur sur http://localhost:{PORT}\n", flush=True)
server = HTTPServer(("0.0.0.0", PORT), MJPEGHandler)
server.timeout = 1

try:
    while True:
        server.handle_request()
except KeyboardInterrupt:
    print("\n⏹️  Arrêt", flush=True)
finally:
    server.server_close()
    if ffmpeg_proc:
        ffmpeg_proc.terminate()

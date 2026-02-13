#!/usr/bin/env python3
"""
WebRTC low-latency RTSP bridge using aiortc.
- Serves a tiny web client from ./webrtc/index.html
- Signaling over WebSocket at /ws
"""

import asyncio
import json
import os
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaPlayer

RTSP_URL = os.environ.get(
    "RTSP_URL",
    "rtsp://admin:ros2_2025@100.73.141.53:8554/h264Preview_01_main",
)
RTSP_TRANSPORT = os.environ.get("RTSP_TRANSPORT", "tcp")
VIDEO_SIZE = os.environ.get("VIDEO_SIZE", "")
HOST = "0.0.0.0"
PORT = 8091
STATIC_DIR = os.path.dirname(__file__)

pcs = set()


def build_player():
    # Try TCP first for reliability; switch to udp if needed.
    options = {
        "rtsp_transport": RTSP_TRANSPORT,
        "fflags": "nobuffer",
        "flags": "low_delay",
        "max_delay": "0",
        "reorder_queue_size": "0",
        "stimeout": "5000000",
    }
    if VIDEO_SIZE:
        options["video_size"] = VIDEO_SIZE

    return MediaPlayer(RTSP_URL, format="rtsp", options=options)


async def index(request):
    return web.FileResponse(os.path.join(STATIC_DIR, "index.html"))


async def app_js(request):
    return web.FileResponse(os.path.join(STATIC_DIR, "app.js"))


async def websocket_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)

    pc = RTCPeerConnection()
    pcs.add(pc)

    print("WebRTC: client connecté", flush=True)

    player = build_player()
    if player.video:
        pc.addTrack(player.video)
        print("WebRTC: piste vidéo ajoutée", flush=True)
    else:
        await ws.send_json({"type": "error", "message": "RTSP: pas de piste video"})
        print("WebRTC: pas de piste vidéo", flush=True)

    if player.audio:
        pc.addTrack(player.audio)
        print("WebRTC: piste audio ajoutée", flush=True)
    else:
        print("WebRTC: pas de piste audio", flush=True)

    @pc.on("icegatheringstatechange")
    async def on_icegatheringstatechange():
        print(f"ICE gathering: {pc.iceGatheringState}", flush=True)

    @pc.on("iceconnectionstatechange")
    async def on_iceconnectionstatechange():
        print(f"ICE connection: {pc.iceConnectionState}", flush=True)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print(f"WebRTC state: {pc.connectionState}", flush=True)
        if pc.connectionState in ("failed", "closed", "disconnected"):
            await pc.close()
            pcs.discard(pc)

    async for msg in ws:
        if msg.type == web.WSMsgType.TEXT:
            data = json.loads(msg.data)
            if data.get("type") == "offer":
                print("WebRTC: offer reçue", flush=True)
                offer = RTCSessionDescription(sdp=data["sdp"], type=data["type"])
                await pc.setRemoteDescription(offer)
                answer = await pc.createAnswer()
                await pc.setLocalDescription(answer)
                await ws.send_json({"type": "answer", "sdp": pc.localDescription.sdp})
                print("WebRTC: answer envoyée", flush=True)

    return ws


async def on_shutdown(app):
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()


def main():
    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_get("/app.js", app_js)
    app.router.add_get("/ws", websocket_handler)
    app.on_shutdown.append(on_shutdown)

    web.run_app(app, host=HOST, port=PORT)


if __name__ == "__main__":
    main()

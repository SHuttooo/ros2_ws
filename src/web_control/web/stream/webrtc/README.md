# WebRTC RTSP Bridge

Ce dossier expose le flux RTSP en WebRTC (latence très basse).

## Lancer

```bash
cd /home/matthieu/ros2_ws/src/web_control/web/stream/webrtc
python3 -m pip install -r requirements.txt
python3 webrtc_server.py
```

Puis ouvrir : `http://localhost:8091`

## Remarques

- Le transport RTSP par défaut est `tcp` (plus fiable). Tu peux passer en `udp` via variable d'environnement.
- Pour forcer une résolution, utilise `VIDEO_SIZE` (ex: `1920x1080`).

### Exemple (UDP + 1080p)

```bash
RTSP_TRANSPORT=udp VIDEO_SIZE=1920x1080 python3 webrtc_server.py
```

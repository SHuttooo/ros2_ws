#!/usr/bin/env python3
"""
Node ROS2 pour contrôler la caméra Reolink (PTZ + Lampe IR)
Ce node simule le côté robot et écoute les topics ROS2.

Topics écoutés:
  - /camera/ptz (geometry_msgs/Point) : Contrôle pan/tilt
  - /camera/zoom (std_msgs/Float32) : Contrôle zoom
  - /camera/light (std_msgs/Bool) : Contrôle lampe IR
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32, Bool
import requests
import time

class CameraControlNode(Node):
    def __init__(self):
        super().__init__('camera_control_node')
        
        # Paramètres caméra Reolink
        self.camera_ip = "10.42.0.188"
        self.camera_user = "admin"
        self.camera_password = "ros2_2025"
        self.camera_url = f"http://{self.camera_ip}/api.cgi"
        
        # Token de session
        self.token = None
        self.token_expiry = 0
        
        # État interne
        self.ptz_speed = 32
        self.zoom_speed = 4
        self.zoom_max = 28
        self.light_state = False
        self.ptz_active = False
        
        # Subscriptions ROS2
        self.ptz_sub = self.create_subscription(Point, '/camera/ptz', self.ptz_callback, 10)
        self.zoom_sub = self.create_subscription(Float32, '/camera/zoom', self.zoom_callback, 10)
        self.light_sub = self.create_subscription(Bool, '/camera/light', self.light_callback, 10)
        self.focus_sub = self.create_subscription(Float32, '/camera/focus', self.focus_callback, 10)
        self.autofocus_sub = self.create_subscription(Bool, '/camera/autofocus', self.autofocus_callback, 10)
        self.alert_sub = self.create_subscription(Bool, '/camera/alert', self.alert_callback, 10)
        self.robot_volume_sub = self.create_subscription(Float32, '/robot/volume', self.robot_volume_callback, 10)
        
        # Timer pour rafraîchir le token
        self.create_timer(300.0, self.refresh_token)
        
        self.get_logger().info(f"🎥 Camera Control Node démarré")
        self.get_logger().info(f"   IP: {self.camera_ip}")
        self.get_logger().info(
            "   Topics: /camera/ptz, /camera/zoom, /camera/light, /camera/focus, /camera/autofocus, /camera/alert, /robot/volume"
        )
        
        # Connexion initiale
        self.login()
        
        # Activer l'autofocus par défaut
        self.enable_autofocus()
    
    def login(self):
        """Se connecte à la caméra et obtient un token"""
        try:
            url = f"{self.camera_url}?cmd=Login"
            payload = [{
                "cmd": "Login",
                "param": {
                    "User": {
                        "userName": self.camera_user,
                        "password": self.camera_password
                    }
                }
            }]
            
            response = requests.post(url, json=payload, timeout=5)
            
            if response.status_code == 200:
                data = response.json()
                if data and len(data) > 0 and data[0].get("code") == 0:
                    self.token = data[0]["value"]["Token"]["name"]
                    self.token_expiry = time.time() + 3600
                    self.get_logger().info(f"✅ Connexion réussie (token: {self.token[:10]}...)")
                    return True
                else:
                    error = data[0].get("error", {}) if data else {}
                    self.get_logger().error(f"❌ Login échoué: {error.get('detail', 'erreur')}")
            else:
                self.get_logger().error(f"❌ Login HTTP {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"❌ Erreur login: {e}")
        return False
    
    def refresh_token(self):
        """Rafraîchit le token avant expiration"""
        if time.time() > self.token_expiry - 300:
            self.get_logger().info("🔄 Rafraîchissement token...")
            self.login()
    
    def send_camera_command(self, cmd, params, action=None, payload_override=None):
        """Envoie une commande à la caméra"""
        if not self.token or time.time() > self.token_expiry:
            self.get_logger().warn("Token expiré, reconnexion...")
            if not self.login():
                return False
        
        try:
            if payload_override is not None:
                payload = payload_override
            else:
                payload = [{"cmd": cmd, "param": params}]
                if action is not None:
                    payload[0]["action"] = action
            url = f"{self.camera_url}?cmd={cmd}&token={self.token}"
            response = requests.post(url, json=payload, timeout=2)
            
            if response.status_code == 200:
                data = response.json()
                if data and len(data) > 0:
                    if data[0].get("code") == 0:
                        return True
                    else:
                        error = data[0].get("error", {})
                        detail = error.get("detail", "erreur")
                        if "login" in detail.lower():
                            self.get_logger().warn("Session expirée, reconnexion...")
                            if self.login():
                                return self.send_camera_command(cmd, params)
                        self.get_logger().warn(f"Commande {cmd} échouée: {detail}")
            else:
                self.get_logger().warn(f"Commande {cmd} HTTP {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"Erreur {cmd}: {e}")
        return False

    def get_audio_volume(self):
        """Récupère le volume audio actuel (0-100)"""
        if not self.token or time.time() > self.token_expiry:
            if not self.login():
                return None

        try:
            payload = [{"cmd": "GetAudioCfg", "param": {"channel": 0}}]
            url = f"{self.camera_url}?cmd=GetAudioCfg&token={self.token}"
            response = requests.post(url, json=payload, timeout=2)
            if response.status_code == 200:
                data = response.json()
                if data and data[0].get("code") == 0:
                    return data[0]["value"]["AudioCfg"].get("volume")
        except Exception:
            pass

        return None
    
    def ptz_callback(self, msg):
        """Contrôle PTZ"""
        # Si toutes les valeurs sont à 0, c'est un stop
        if abs(msg.x) < 0.01 and abs(msg.y) < 0.01 and abs(msg.z) < 0.01:
            if self.ptz_active:
                self.stop_ptz()
            return
        
        # Sinon, exécuter le mouvement demandé
        operation = None
        if abs(msg.x) > 0.01:
            operation = "Up" if msg.x > 0 else "Down"
        elif abs(msg.y) > 0.01:
            operation = "Right" if msg.y > 0 else "Left"
        elif abs(msg.z) > 0.01:
            operation = "ZoomInc" if msg.z > 0 else "ZoomDec"
        
        if operation:
            self.execute_ptz(operation)
            self.ptz_active = True
    
    def execute_ptz(self, operation):
        """Exécute une commande PTZ"""
        speed = self.zoom_speed if "Zoom" in operation else self.ptz_speed
        params = {"channel": 0, "op": operation, "speed": speed}
        
        if self.send_camera_command("PtzCtrl", params):
            self.get_logger().info(f"🎮 PTZ: {operation} (speed={speed})")
    
    def stop_ptz(self):
        """Arrête le PTZ"""
        params = {"channel": 0, "op": "Stop"}
        if self.send_camera_command("PtzCtrl", params):
            self.get_logger().info("⏹️  PTZ: Stop")
        self.ptz_active = False
    
    def zoom_callback(self, msg):
        """Contrôle zoom absolu (0-100%)"""
        # Clamp la valeur entre 0 et 100
        zoom_percent = max(0.0, min(100.0, msg.data))

        # Mapper la plage 0-100% vers la plage caméra (0-zoom_max)
        zoom_pos = int(round((zoom_percent / 100.0) * self.zoom_max))
        zoom_pos = max(0, min(self.zoom_max, zoom_pos))

        # Utiliser StartZoomFocus avec position absolue
        params = {"ZoomFocus": {"channel": 0, "op": "ZoomPos", "pos": zoom_pos}}

        if self.send_camera_command("StartZoomFocus", params):
            self.get_logger().info(f"🔍 Zoom: {int(zoom_percent)}% (pos={zoom_pos})")
    
    def light_callback(self, msg):
        """Contrôle lumière blanche"""
        state = 1 if msg.data else 0
        params = {"WhiteLed": {"channel": 0, "state": state}}
        
        if self.send_camera_command("SetWhiteLed", params):
            self.light_state = msg.data
            self.get_logger().info(f"💡 Lumière blanche: {'On' if msg.data else 'Off'}")

    def alert_callback(self, msg):
        """Déclenche l'alarme/sirène de la caméra"""
        if not msg.data:
            return

        state = 1

        payload = [{
            "cmd": "AudioAlarm",
            "action": 0,
            "param": {
                "alarm": {
                    "channel": 0,
                    "state": state
                }
            }
        }]

        if self.send_camera_command("AudioAlarm", params=None, payload_override=payload):
            self.get_logger().info("🚨 Alerte caméra: ON")
            return

        fallback = {"Siren": {"channel": 0, "state": state}}
        if self.send_camera_command("SetSiren", fallback):
            self.get_logger().info(f"🚨 Alerte caméra (fallback): {'ON' if msg.data else 'OFF'}")
            return

        if msg.data:
            volume = self.get_audio_volume()
            if volume is None:
                volume = 50
            params = {"AudioCfg": {"channel": 0, "volume": int(volume), "test": 1}}
            if self.send_camera_command("SetAudioCfg", params, action=0):
                self.get_logger().info("🚨 Sound Test (AudioCfg) envoyé")
                return
            self.get_logger().warn("⚠️  Sound Test (AudioCfg) non supporté")
            return

        self.get_logger().warn("⚠️  Alerte caméra: commande non supportée")

    def robot_volume_callback(self, msg):
        """Règle le volume du haut-parleur caméra (0-100)"""
        volume = max(0.0, min(100.0, msg.data))
        params = {"AudioCfg": {"channel": 0, "volume": int(volume)}}
        if self.send_camera_command("SetAudioCfg", params, action=0):
            self.get_logger().info(f"🔊 Volume caméra: {int(volume)}%")
    
    def enable_autofocus(self):
        """Active l'autofocus automatique"""
        params = {"AutoFocus": {"channel": 0, "disable": 0, "afType": 0}}
        if self.send_camera_command("SetAutoFocus", params):
            self.get_logger().info("🎯 Autofocus activé")
            return
        fallback = {"ZoomFocus": {"channel": 0, "op": "AutoFocus"}}
        if self.send_camera_command("StartZoomFocus", fallback):
            self.get_logger().info("🎯 Autofocus activé (fallback)")
    
    def focus_callback(self, msg):
        """Contrôle focus manuel (0=auto, 1-28=position manuelle)"""
        focus_pos = max(0.0, min(28.0, msg.data))
        
        # Focus manuel
        # D'abord désactiver autofocus
        params = {"AutoFocus": {"channel": 0, "disable": 1, "afType": 0}}
        self.send_camera_command("SetAutoFocus", params)
        
        # Puis définir la position de focus
        params = {"ZoomFocus": {"channel": 0, "op": "FocusPos", "pos": int(focus_pos)}}
        if self.send_camera_command("StartZoomFocus", params):
            self.get_logger().info(f"🎯 Focus manuel: {int(focus_pos)}")
            return
        
        # Fallback: certains modèles attendent un champ focus.pos
        params = {"ZoomFocus": {"channel": 0, "focus": {"pos": int(focus_pos)}}}
        if self.send_camera_command("StartZoomFocus", params):
            self.get_logger().info(f"🎯 Focus manuel (fallback): {int(focus_pos)}")

    def autofocus_callback(self, msg):
        """Active/désactive l'autofocus via topic dédié"""
        if msg.data:
            self.enable_autofocus()
        else:
            params = {"AutoFocus": {"channel": 0, "disable": 1, "afType": 0}}
            if self.send_camera_command("SetAutoFocus", params):
                self.get_logger().info("🎯 Autofocus désactivé")


def main(args=None):
    rclpy.init(args=args)
    node = CameraControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Arrêt du node...")
    finally:
        if node.ptz_active:
            node.stop_ptz()
        if node.light_state:
            params = {"IrLights": {"channel": 0, "state": "Off"}}
            node.send_camera_command("SetIrLights", params)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

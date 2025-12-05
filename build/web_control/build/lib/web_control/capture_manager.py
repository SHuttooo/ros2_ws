import cv2
import os
import time
from datetime import datetime
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class CaptureManager:
    def __init__(self, node, gallery_path):
        self.node = node
        self.gallery_path = gallery_path
        self.bridge = CvBridge()
        self.latest_image = None
        self.recording = False
        self.video_writer = None
        
        # Abonnement au flux caméra pour capturer
        self.sub = self.node.create_subscription(
            Image, '/image_raw', self.image_callback, 10)

    def image_callback(self, msg):
        try:
            # Conversion ROS Image -> OpenCV
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Gestion enregistrement vidéo
            if self.recording and self.video_writer:
                self.video_writer.write(self.latest_image)
        except Exception as e:
            self.node.get_logger().error(f"Erreur conversion image: {e}")

    def take_photo(self):
        if self.latest_image is None:
            return False, "Pas d'image reçue"
        
        filename = f"photo_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
        path = os.path.join(self.gallery_path, filename)
        cv2.imwrite(path, self.latest_image)
        self.node.get_logger().info(f"Photo sauvegardée: {path}")
        return True, path

    def start_video(self):
        if self.recording:
            return False, "Déjà en cours"
        if self.latest_image is None:
            return False, "Pas d'image pour initier la vidéo"
            
        filename = f"video_{datetime.now().strftime('%Y%m%d_%H%M%S')}.avi"
        path = os.path.join(self.gallery_path, filename)
        
        height, width, _ = self.latest_image.shape
        # Codec MJPG
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.video_writer = cv2.VideoWriter(path, fourcc, 20.0, (width, height))
        
        self.recording = True
        self.node.get_logger().info(f"Début enregistrement: {path}")
        return True, "Enregistrement démarré"

    def stop_video(self):
        if not self.recording:
            return False, "Pas d'enregistrement en cours"
        
        self.recording = False
        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None
        
        self.node.get_logger().info("Fin enregistrement")
        return True, "Enregistrement arrêté"

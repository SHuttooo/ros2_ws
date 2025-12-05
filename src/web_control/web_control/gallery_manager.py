import os
import json
from std_msgs.msg import String

class GalleryManager:
    def __init__(self, node, gallery_path):
        self.node = node
        self.gallery_path = gallery_path
        self.pub = self.node.create_publisher(String, '/ui/gallery_files', 10)
        
        # Timer pour mettre à jour la galerie (1Hz)
        self.timer = self.node.create_timer(1.0, self.publish_gallery)

    def publish_gallery(self):
        if not os.path.exists(self.gallery_path):
            return

        # Liste des fichiers jpg et avi
        files = [f for f in os.listdir(self.gallery_path) if f.endswith(('.jpg', '.avi', '.png'))]
        # Tri par date (plus récent en premier)
        files.sort(key=lambda x: os.path.getmtime(os.path.join(self.gallery_path, x)), reverse=True)
        
        msg = String()
        msg.data = json.dumps(files)
        self.pub.publish(msg)

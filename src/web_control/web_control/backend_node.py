import rclpy
from rclpy.node import Node
import os
import threading
import http.server
import socketserver
import shutil
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point # <--- Changement ici (Twist -> Point)
from std_msgs.msg import String     # <--- Pour la suppression

# Modules internes
from web_control.capture_manager import CaptureManager
from web_control.gallery_manager import GalleryManager

PORT_WEB = 8000

class QuietHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def copyfile(self, source, outputfile):
        try:
            super().copyfile(source, outputfile)
        except (BrokenPipeError, ConnectionResetError):
            pass

class ThreadedHTTPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True

class WebBackend(Node):
    def __init__(self):
        super().__init__('web_backend')
        
        package_share = get_package_share_directory('web_control')
        self.web_dir = os.path.join(package_share, 'web')
        self.gallery_dir = os.path.join(self.web_dir, 'gallery')
        
        if not os.path.exists(self.gallery_dir):
            os.makedirs(self.gallery_dir)

        self.capture_mgr = CaptureManager(self, self.gallery_dir)
        self.gallery_mgr = GalleryManager(self, self.gallery_dir)

        # Services
        self.srv_photo = self.create_service(Trigger, '/camera/take_photo', self.cb_take_photo)
        self.srv_start_vid = self.create_service(Trigger, '/camera/start_video', self.cb_start_video)
        self.srv_stop_vid = self.create_service(Trigger, '/camera/stop_video', self.cb_stop_video)

        # Subscribers (Modifiés en Point)
        self.create_subscription(Point, '/robot/cmd_vel_buttons', self.cb_cmd_vel, 10)
        self.create_subscription(Point, '/camera/ptz', self.cb_ptz, 10)
        
        # Subscriber pour suppression
        self.create_subscription(String, '/camera/delete_image', self.cb_delete_image, 10)

        self.httpd = None
        self.server_thread = None

        self.get_logger().info(f"Backend prêt. Web Port: {PORT_WEB}")
        self.start_web_server()

    def start_web_server(self):
        try:
            os.chdir(self.web_dir)
            self.httpd = ThreadedHTTPServer(("", PORT_WEB), QuietHandler)
            self.server_thread = threading.Thread(target=self.httpd.serve_forever)
            self.server_thread.daemon = True
            self.server_thread.start()
        except OSError as e:
            self.get_logger().error(f"Erreur serveur web: {e}")

    # Callbacks (Juste pour debug ou hardware)
    def cb_cmd_vel(self, msg):
        # msg.x = Avant/Arrière, msg.y = Gauche/Droite
        pass

    def cb_ptz(self, msg):
        pass

    # Logique de suppression
    def cb_delete_image(self, msg):
        filename = msg.data
        # Sécurité simple pour ne pas sortir du dossier
        if ".." in filename or filename.startswith("/"):
            return
            
        path = os.path.join(self.gallery_dir, filename)
        if os.path.exists(path):
            try:
                os.remove(path)
                self.get_logger().info(f"Fichier supprimé: {filename}")
                # Force la mise à jour de la galerie pour le client
                self.gallery_mgr.publish_gallery()
            except Exception as e:
                self.get_logger().error(f"Erreur suppression: {e}")

    def cb_take_photo(self, request, response):
        success, msg = self.capture_mgr.take_photo()
        response.success = success
        response.message = msg
        return response

    def cb_start_video(self, request, response):
        success, msg = self.capture_mgr.start_video()
        response.success = success
        response.message = msg
        return response

    def cb_stop_video(self, request, response):
        success, msg = self.capture_mgr.stop_video()
        response.success = success
        response.message = msg
        return response

    def stop_server(self):
        if self.httpd:
            self.httpd.shutdown()
            self.httpd.server_close()

def main(args=None):
    rclpy.init(args=args)
    node = WebBackend()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_server()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

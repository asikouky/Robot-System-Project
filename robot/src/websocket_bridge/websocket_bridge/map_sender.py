import asyncio
import os
import json
import subprocess
import threading
import time
import yaml
import base64
from PIL import Image
from common_interfaces.event_types import EventType
import rclpy

#section a changer en fonction de ou est l'image de la map
# __file__ est le chemin du fichier courant (websocket_ros_bridge.py)
current_dir = os.path.dirname(os.path.abspath(__file__))

# Remonter de 3 niveaux pour atteindre la racine du workspace
root_dir = os.path.abspath(os.path.join(current_dir, '..', '..', '..'))

# Construire le chemin complet vers map.jpg
map_path = os.path.join(root_dir, 'src','map_generated', 'map_test.jpg')
pgm_path = '/home/inf995/INF3995-104/robot/src/merged_map.pgm'
yaml_path  = '/home/inf995/INF3995-104/robot/src/merged_map.yaml'
script_path = '/home/inf995/INF3995-104/robot/src/save_map.sh'


class MapSender:
    def __init__(self, get_websocket_callback):
        self.pgm_path = pgm_path
        self.yaml_path = yaml_path
        self.get_websocket = get_websocket_callback  # fonction qui retourne le websocket actif
        self.logger = rclpy.logging.get_logger("MapSender")

        # Pour le mode continu
        self.running = False
        self.thread = None

    def convert_pgm_to_png(self):
        base_name = os.path.splitext(self.pgm_path)[0]
        png_path = base_name + ".png"

        if not os.path.exists(self.pgm_path):
            raise FileNotFoundError(f"{self.pgm_path} introuvable.")
        
        with Image.open(self.pgm_path) as img:
            img.save(png_path)

        return png_path

    def read_map_metadata(self):
        if not os.path.exists(self.yaml_path):
            raise FileNotFoundError(f"{self.yaml_path} introuvable.")
        
        with open(self.yaml_path, 'r') as f:
            metadata = yaml.safe_load(f)

        return metadata

    async def send_map_update(self):

        websocket = self.get_websocket()
        if websocket is None:
            return  # Aucun client connecté

        try:
            ip = websocket.remote_address[0] if websocket.remote_address else "unknown"
            metadata = self.read_map_metadata()
            png_path = self.convert_pgm_to_png()

            with open(png_path, 'rb') as f:
                image_base64 = base64.b64encode(f.read()).decode('utf-8')

            message = {
                "eventType": EventType.MAP.value,
                "type": "map_image",
                "data": {
                    "ip": ip,
                    "image": image_base64,
                    "metadata": metadata
                }
            }

            await websocket.send(json.dumps(message))

        except Exception as e:
            print(f"[MapSender] ❌ Erreur d’envoi de la map : {e}")
            
    def start_sending(self, interval_seconds=2):
            if not self.running:
                self.running = True
                self.thread = threading.Thread(target=self._send_loop, args=(interval_seconds,), daemon=True)
                self.thread.start()
                self.logger.info("🔁 Envoi de la carte en continu démarré")

    def stop_sending(self):
        if self.running:
            self.logger.info("⛔ Arrêt de l'envoi continu de la carte")
            self.running = False
            if self.thread:
                self.thread.join()
                self.thread = None

    def save_merged_map(self):
        self.logger.info("💾 Sauvegarde via tmux...")

        try:
            command = f"bash -c 'source /opt/ros/humble/setup.bash && bash {script_path}'"
            subprocess.run(["tmux", "new-session", "-d", command], check=True)
            self.logger.info("✅ Script lancé dans une session tmux")
        except subprocess.CalledProcessError as e:
            self.logger.error(f"❌ Erreur avec tmux : {e}")


    def _send_loop(self, interval):
        while self.running:
            try:
                self.save_merged_map()  # sauvegarde actualisée de la carte
                asyncio.run(self.send_map_update())
            except Exception as e:
                self.logger.error(f"❌ Erreur dans la boucle d’envoi de map : {e}")
            time.sleep(interval)
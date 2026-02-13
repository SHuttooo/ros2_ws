# 🤖 AGRI-BOT - Interface de Contrôle Distante

**Surveillance intelligente des cultures à distance** | Robot agricole autonome | Polytech Sorbonne | Équipe ROB4

Plateforme complète de pilotage à distance pour **AGRI-BOT**, un robot agricole capable de surveiller la santé des cultures en serre et en verger, directement depuis le campus de Jussieu via une connexion ROS2 + ZENOH reliant Saint-Cyr à Jussieu.

---

## 📋 Table des matières

1. [À propos du projet](#-à-propos-du-projet)
2. [Cahier des Charges & Spécifications](#-cahier-des-charges--spécifications)
3. [Architecture système](#️-architecture-système)
4. [Installation](#-installation)
5. [Configuration](#️-configuration)
6. [Lancement](#-lancement)
7. [Guide d'utilisation](#-guide-dutilisation)
8. [Pages et fonctionnalités](#-pages-et-fonctionnalités)
9. [API & Topics ROS2](#-api--topics-ros2)
10. [Scénarios d'Utilisation](#-scénarios-dutilisation)
11. [Démarrage Rapide](#-démarrage-rapide)
12. [Troubleshooting](#-troubleshooting)
13. [Structure du projet](#-structure-du-projet)

---

## 🌾 À propos du projet

### La Mission

**AGRI-BOT** permet de surveiller **la santé des cultures** en serre connectée et en verger de Saint-Cyr **directement depuis Jussieu**. Grâce à cette base mobile modulaire, nous remplaçons les capteurs fixes par un système agile capable de détecter les besoins ou les maladies des plantes en temps réel.

### Capacités du Robot

✅ **Exploration Tout-Terrain**: Transmission 4x4 adaptée aux sols irréguliers du verger  
✅ **Vision Haute Précision**: Caméra PTZ avec zoom optique pour inspection détaillée  
✅ **Taille Ajustable**: Bras pantographe (30 cm → 1m20) pour adapter la vue aux cultures  
✅ **Navigation Autonome**: ROS2 + SLAM pour déplacement sans intervention  
✅ **Stockage Hors-Ligne**: Enregistrement local des données en cas de perte réseau  

### Conception

| Élément | Fonction | Justification |
|---------|----------|---------------|
| **Base Curt Mini** | Locomotion | Étanchéité et franchissement en milieu humide |
| **Système Pantographe** | Élévation | Solution mécanique fiable, stable et économique |
| **Cerveau ROS2** | Intelligence | Navigation autonome, fusion capteurs, évitement |

### Équipe

- **Fatoumata DIALLO**
- **Pierre-Louis PASUTTO**
- **Matthieu VINET**

Encadrés par: **Aline BAUDRY**

---

## 📋 Cahier des Charges & Spécifications

### Fonctions de Service Principales (FP)

| Fonction | Description | Critère de validation |
|----------|-------------|----------------------|
| **FP1** | Suivre un parcours sur demande | Précision ±0.2m (extérieur), ±0.05m (intérieur) |
| **FP2** | Télé-opérer le robot (Commande + Visualisation) | Latence < 0.5 sec, résolution 480p min |
| **FP3** | Filmer/Photographier les plantes | Full HD, distance mise au point 10-50 cm |
| **FP4** | Se déplacer dans les zones non-couvertes | Suivi hors-ligne, stockage 1 Go de contenu |

### Contraintes Critiques (FC)

| Contrainte | Description | Priorité |
|-----------|-------------|----------|
| **FC1** | Identification utilisateur sécurisée | F0 (Obligatoire) |
| **FC2** | Navigation autonome sur alerte | F4 (Facultatif) |
| **FC3** | Transiter les données, upload web | F0 (Obligatoire) |
| **FC4** | Éviter les obstacles imprévus | F0 (Obligatoire) - Détection 10-50 cm, >95% fiabilité |
| **FC5** | Open-source | F0 (Obligatoire) |
| **FC6** | Résistance environnementale | F0 (Obligatoire) - IP68, pluie/éclaboussures/poussière |
| **FC7** | Autonomie batterie longue durée | F1 (Très prioritaire) - 3-4h min |
| **FC8** | S'adapter aux plantes filmées | F1 (Très prioritaire) |
| **FC9** | Franchir portes basses (serre) | F0 (Obligatoire) - Hauteur <30 cm repliée |
| **FC10** | S'adapter aux terrains changeants | F0 (Obligatoire) - Gravier, herbe, marche max 4cm, pente max 5° |
| **FC11** | Connectivité WiFi limitée | F1 (Très prioritaire) - Fonctionnalité hors-ligne |

### Caractéristiques du Robot AGRI-BOT

#### 🚧 Configuration Actuelle vs Configuration Finale

**⚠️ ÉTAT ACTUEL (Février 2026):**

Nous utilisons actuellement un **TurtleBot3 Burger** comme **plateforme de simulation** pour:
- ✅ Développer et valider l'interface web de contrôle
- ✅ Tester l'architecture ROS2 + ZENOH multi-site (Jussieu ↔ Saint-Cyr)
- ✅ Calibrer les commandes de mouvement (`/cmd_vel`, `/camera/ptz`)
- ✅ Valider le streaming vidéo MJPEG
- ✅ Développer les algorithmes de navigation et SLAM

Le TurtleBot3 **simule temporairement** le Curt Mini en attendant sa livraison. La compatibilité ROS2 garantit une migration transparente.

---

**🎯 CONFIGURATION FINALE (après réception du matériel):**

#### Base Mobile: **Curt Mini 4×4** (Fraunhofer IPA)

**Robot final AGRI-BOT** comprendra:

<div align="center">
  <img src="projet-documentation/robot-illustration.png" alt="Illustration du robot AGRI-BOT final" width="600"/>
  <p><em>Visualisation du robot AGRI-BOT final avec bras pantographe, caméra PTZ et LiDAR</em></p>
</div>

```
┌─────────────────────────────────────────┐
│         🤖 AGRI-BOT - Curt Mini         │
│                                         │
│  ┌────────────────────────────────┐     │
│  │  📡 LiDAR 2D (SLAM + obstacles)│     │ ← Détection et cartographie
│  └────────────────────────────────┘     │
│              ▲                          │
│              │                          │
│  ┌────────────────────────────────┐     │
│  │  📷 Caméra PTZ (Pan-Tilt-Zoom) │     │ ← Inspection plantes
│  │  (sur bras pantographe)        │     │
│  └────────────────────────────────┘     │
│              ▲                          │
│              │                          │
│  ┌────────────────────────────────┐     │
│  │  🦾 Bras Pantographe           │     │ ← Hauteur ajustable 30cm → 1m20
│  │  (système mécanique)           │     │
│  └────────────────────────────────┘     │
│              ▲                          │
│              │                          │
│  ┌────────────────────────────────┐     │
│  │  🚗 Base Curt Mini 4×4         │     │ ← Locomotion tout-terrain
│  │  - IP68 (étanche)              │     │
│  │  - 15.5 cm garde au sol        │     │
│  │  - Charge 30 kg                │     │
│  │  - Châssis à bascule           │     │
│  └────────────────────────────────┘     │
│                                         │
│  📦 ROS2 Jazzy                          │
│  🔋 Batterie (3-4h autonomie)           │
│  📶 WiFi/4G + GPS RTK (extérieur)       │
└─────────────────────────────────────────┘
```

**Caractéristiques Curt Mini 4×4**:
- ✅ Étanchéité excellente: **IP68** moteurs + châssis résistant pluie/éclaboussures/poussière
- ✅ Franchissement supérieur: **15.5 cm** garde au sol + châssis à bascule
- ✅ Charge utile élevée: **30 kg** (bras, caméra, batterie, capteurs, LiDAR)
- ✅ Autonomie: **3-4 heures**
- ✅ Compatibilité ROS2 Jazzy complète
- ✅ Dimensions: **54 cm (L) × 36 cm (H) × 252 mm (l)** - Passe portes serre
- ✅ Vitesse linéaire: **~0.15 m/s** (adapté à l'inspection)

**Équipements à intégrer sur le Curt Mini**:

> **🚧 EN CONSTRUCTION** - Intégration prévue après réception du robot Curt Mini. Le TurtleBot3 actuel ne possède pas ces équipements.

1. **Bras pantographe** (30 cm replié → 1m20 déployé) - Conception mécanique en cours
2. **Caméra PTZ** (Pan-Tilt-Zoom) - Montée sur le bras
3. **LiDAR 2D** - Monté sur le sommet pour SLAM et évitement d'obstacles
4. **GPS RTK** - Pour navigation extérieure précise (±5 cm)
5. **IMU + Odomètres** - Fusion capteurs pour localisation
6. **Éclairage LED** - Pour inspection en conditions faibles luminosité

**Alternatives écartées**:
- Jackal (Clearpath): IP62 insuffisant pour humidité serre, trop coûteux (26k€)
- Scout Mini (Agilex): IP22 inadapté, trop sensible à l'humidité (7k€ mais non-fiable)
- Chenilles: Trop dommageantes pour sols agricoles
- Drones: Risque collision en serre confinée, complexité réglementaire

---

#### 🦾 Bras Pantographe (Robot Final en théorie)

<div align="center">
  <img src="projet-documentation/bras-illustration.png" alt="Bras pantographe du robot AGRI-BOT" width="500"/>
  <p><em>Bras pantographe à hauteur variable (30 cm → 1m20) pour inspection des cultures</em></p>
</div>

**Spécifications pour montage sur Curt Mini**:
- **Plage de hauteur**: 30 cm (replié) → 1m20 (déployé)
- **Charge utile**: Caméra PTZ + capteurs additionnels (~2-3 kg)
- **Conception**: Solution mécanique stable et fiable (pas de vérin pneumatique fragile)
- **Contrôle**: 1 moteur servo pour déploiement progressif
- **Vitesse**: Lente acceptable (priorité: stabilité image)
- **Matériau**: Aluminium ou composite léger
- **Fixation**: Sur châssis supérieur Curt Mini

> **🚧 EN CONSTRUCTION** - Conception mécanique en cours. Le bras sera intégré après réception du Curt Mini. Le TurtleBot3 actuel ne possède pas de bras (simulation software uniquement).

---

#### 📷 Caméra de Supervision (Configuration Finale)

**Système final**: Caméra PTZ professionnelle montée sur bras pantographe

**Système actuel (TurtleBot3)**: ros2 image_tools cam2image + USB camera V4L2

**Caractéristiques caméra finale**:
- Résolution: 1280×720 (Full HD disponible)
- Fréquence: 30 FPS
- Type: Caméra PTZ (Pan-Tilt-Zoom)
- Distance focale: 10 cm à 50 cm (inspection détaillée plantes)
- Topic ROS2: `/image_raw` (type: sensor_msgs/Image)
- Encodage: MJPEG via web_video_server

#### Localisation & Navigation

**Capteurs**:
- GPS RTK: Positionnement extérieur (verger) ±5 cm
- LiDAR 2D: SLAM intérieur (serre) - Détection obstacles
- IMU: Stabilisation et odométrie
- Odomètres: Roues encodeur

**Algorithmes**:
- SLAM (Simultaneous Localization and Mapping): Cartographie temps réel
- Navigation autonome: ROS2 nav2 stack
- Évitement d'obstacles: Costmaps dynamiques

---

## 🏗️ Architecture système

### Vue d'ensemble avec WebRTC

```
┌───────────────────────────────────────────────────────────────┐
│                     SERVEUR ZEHNO (PC)                        │
│                                                               │
│  ┌────────────────────────────────────────────────────────┐   │
│  │ Navigateur Web (http://localhost:8000)                 │   │
│  │  - app.js: WebRTC client + contrôles ROS2              │   │
│  │  - style.css: UI responsive                            │   │
│  │  - index.html: Structure HTML + video element          │   │
│  └────────────────────────────────────────────────────────┘   │
│           │              │                 │                  │
│           ▼ WS (9090)    ▼ WS (8091)       ▼ HTTP             │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐         │
│  │ rosbridge    │  │ WebRTC       │  │ backend_node │         │
│  │ websocket    │  │ Server       │  │ HTTP Server  │         │
│  │ Port 9090    │  │ Port 8091    │  │ Port 8000    │         │
│  └──────────────┘  └──────────────┘  └──────────────┘         │
│        │                  │                  │                │
│        ▼ ROS2             ▼ RTSP             ▼ File I/O       │
│  ┌──────────────────────────────────────────────────┐         │
│  │   ROS2 Topics: /cmd_vel, /battery_state, etc.   │         │
│  └──────────────────────────────────────────────────┘         │
│                           │                                   │
│                           ▼ RTSP/TCP                          │
│  ┌──────────────────────────────────────────────────┐         │
│  │  Caméra Reolink (RTSP)                           │         │
│  │  rtsp://admin:***@100.73.141.53:8554/...        │         │
│  │  - Vidéo: H.264, 2304x1296, 15fps                │         │
│  │  - Audio: AAC, 16kHz, mono (si activé)           │         │
│  └──────────────────────────────────────────────────┘         │
│                                                               │
└───────────────────────────────────────────────────────────────┘
                         │
                         │ ROS_DOMAIN_ID=42
                         │ (via Tailscale VPN ou réseau local)
                         │
┌───────────────────────────────────────────────────────────────┐
│  RASPBERRY PI - TurtleBot3 Burger (Saint-Cyr)                │
│                                                               │
│  ┌────────────────────────────────────────────────────────┐   │
│  │ turtlebot3_bringup/robot.launch.py                     │   │
│  │  - Moteurs: écoute /cmd_vel                            │   │
│  │  - Capteurs: publie /odom, /imu, /battery_state        │   │
│  └────────────────────────────────────────────────────────┘   │
│                        │                                      │
│                        ▼                                      │
│  ┌────────────────────────────────────────────────────────┐   │
│  │  rosbridge_websocket (port 9090)                       │   │
│  │  - Expose topics ROS2 en JSON WebSocket                │   │
│  └────────────────────────────────────────────────────────┘   │
│                                                               │
└───────────────────────────────────────────────────────────────┘
```

### Architecture WebRTC détaillée

**Streaming vidéo bas latence** :
```
Caméra Reolink
  ├─ RTSP/TCP → rtsp://100.73.141.53:8554/h264Preview_01_main
  │   └─ Tunnel socat (si nécessaire)
  │
  ▼
WebRTC Server (Python aiortc)
  ├─ MediaPlayer: décode flux RTSP
  │   ├─ Vidéo: H.264 → track vidéo
  │   └─ Audio: AAC → track audio (si activé)
  │
  ├─ WebSocket (port 8091): signalisation WebRTC
  │   └─ Échange SDP offer/answer avec client
  │
  └─ RTC Peer Connection
      └─ Envoie tracks vidéo/audio au navigateur
  
  ▼
Navigateur (app.js)
  ├─ WebSocket → ws://localhost:8091/ws
  │   └─ Négocie connexion WebRTC (SDP)
  │
  ├─ RTCPeerConnection
  │   ├─ Reçoit track vidéo
  │   └─ Reçoit track audio (optionnel)
  │
  ├─ Video Element (<video id="cameraFeed">)
  │   └─ Affiche le flux en temps réel (~100-200ms latence)
  │
  ├─ Canvas API
  │   └─ Capture screenshots pour photos
  │
  └─ MediaRecorder API
      └─ Enregistre vidéos en WebM
```

**Capture photo/vidéo** :
```
Bouton 📸 Photo
  ▼
canvas.drawImage(videoElement)
  ▼
canvas.toBlob() → JPEG
  ▼
fetch POST /upload_photo
  ▼
backend_node sauvegarde → ~/robot_gallery/photo_*.jpg

Bouton 🔴 REC
  ▼
MediaRecorder(videoStream, {mimeType: 'video/webm'})
  ▼
recordedChunks.push(data)
  ▼
Blob → WebM
  ▼
fetch POST /upload_video
  ▼
backend_node sauvegarde → ~/robot_gallery/video_*.webm
```

### Architecture multi-site détaillée

**Connexion réseau Jussieu ↔ Saint-Cyr**:
- 🔄 **Tailscale VPN** (solution testée actuellement) - VPN mesh peer-to-peer
- 🔒 **VPN Polytech** (solution de production) - VPN institutionnel configuré par l'équipe informatique
- 🌐 ZENOH + ROS_DOMAIN_ID=42 pour découverte automatique des nodes ROS2

```
┌────────────────────────────────────────────────────────────────────┐
│                  CAMPUS JUSSIEU (Serveur Zehno)                    │
│                                                                    │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │         Interface Web (http://<IP>:8000)                     │  │
│  │                                                              │  │
│  │  📋 index.html: Contrôle principal                           │  │
│  │  🗺️ navig.html: Navigation sur carte                         │  │
│  │  🖼️ gallery.html: Galerie photos/vidéos                      │  │
│  │  📟 terminal.html: Logs système                              │  │
│  │  📄 page_presentation.html: Infos projet                     │  │
│  │                                                              │  │
│  │  app.js: Logique ROSLIB.js + contrôles                       │  │
│  │  style.css: UI responsive en vh (9x3 grid)                   │  │
│  └──────────────────────────────────────────────────────────────┘  │
│           │                         │                              │
│           ▼ WS JSON (2 connexions)  ▼ HTTP                         │
│  ┌──────────────────────┐   ┌────────────────────┐                 │
│  │ rosbridge_websocket  │   │ web_video_server   │                 │
│  │ Port 9090            │   │ Port 8080          │                 │
│  └──────────────────────┘   └────────────────────┘                 │
│           │                         │                              │
│           ▼ DDS/ROS2                ▼ HTTP stream                  │
│  ┌──────────────────────────────────────────────┐                  │
│  │        backend_node (port 8000)              │                  │
│  │  - Web server HTTP                           │                  │
│  │  - CaptureManager (photos/vidéos)            │                  │
│  │  - GalleryManager (gestion fichiers)         │                  │
│  │  - Publishers: /ui/trajectory_files          │                  │
│  │  - Publishers: /ui/gallery_files             │                  │
│  │  - Subscribers: /ui/save_trajectory          │                  │
│  └──────────────────────────────────────────────┘                  │
│                        │                                           │
│                        ▼ DDS (ROS_DOMAIN_ID=42)                    │
└────────────────────────────────────────────────────────────────────┘
                         │
            🔗 Tailscale VPN / VPN Polytech 🔗
         (ZENOH Bridge + ROS2 DDS Discovery)
                         │
┌────────────────────────────────────────────────────────────────────┐
│         SITE SAINT-CYR (Raspberry Pi - AGRI-BOT)                   │
│                                                                    │
│  🔄 Configuration actuelle: TurtleBot3 Burger (simulation)         │
│  🎯 Configuration finale: Curt Mini 4×4 + bras + LiDAR             │
│                                                                    │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │ ⚠️  Actuellement: turtlebot3_bringup/robot.launch.py         │  │
│  │  - Moteurs (wheelbase controller)                            │  │
│  │  - Caméra USB (image_raw publisher)                          │  │
│  │  - Capteurs (odom, imu, batterie)                            │  │
│  │                                                              │  │
│  │ 🎯 Configuration finale: curt_mini_bringup/robot.launch.py   │  │
│  │  - Base Curt Mini 4×4 (IP68, 15.5cm garde au sol)            │  │
│  │  - Bras pantographe: /robot/arm_height (30cm → 1m20)         │  │
│  │  - Caméra PTZ sur bras: /camera/ptz (pan/tilt/zoom)          │  │
│  │  - LiDAR 2D: /scan (SLAM + évitement obstacles)              │  │
│  │  - GPS RTK: /gps/fix (navigation extérieure ±5cm)            │  │
│  │  - IMU: /imu/data (stabilisation)                            │  │
│  │  - Batterie: /battery_state (3-4h autonomie)                 │  │
│  └──────────────────────────────────────────────────────────────┘  │
│                        │                                           │
│                        ▼                                           │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │     rosbridge_websocket (port 9090)                          │  │
│  │     Expose topics ROS2 en JSON WebSocket                     │  │
│  └──────────────────────────────────────────────────────────────┘  │
│                                                                    │
└────────────────────────────────────────────────────────────────────┘
```

### Architecture ZENOH

**Commandes du navigateur → Robot**
```
Utilisateur clique Z/Q/S/D (ou OKLM)
      │
      ▼ JSON WebSocket
app.js.sendCmd() → rosbridge (Jussieu:9090)
      │
      ▼ ROS2 Message
/cmd_vel (TwistStamped) ou /camera/ptz (Point)
      │
      ▼ ZENOH/DDS Bridge
rosbridge (Saint-Cyr:9090)
      │
      ▼ ROS2 Subscribers
TurtleBot3 (moteurs, caméra PTZ, bras)
```

**Données du robot → Navigateur**
```
Caméra capture /image_raw (RGB)
      │
      ▼
web_video_server (Saint-Cyr:8080)
      │
      ▼ MJPEG HTTP
Jussieu (Zehno) relaye sur localhost:8080
      │
      ▼ <img src="...">
Navigateur affiche flux vidéo

---

Galerie/Trajets/Logs depuis backend_node
      │
      ▼ ROS2 Topics JSON
/ui/trajectory_files, /ui/gallery_files, /ui/system_logs
      │
      ▼ rosbridge (Jussieu:9090)
      │
      ▼ JSON WebSocket
app.js.updateGallery(), updateTrajectoryList()
      │
      ▼
Navigateur: Galerie, Trajets, Terminal
```

---

## 📲 Pages et fonctionnalités

### 1. **index.html** - Tableau de bord principal

**Sections**:
- 🎥 **Live Feed**: Flux vidéo temps réel (MJPEG 1920×1080)
- 🕹️ **Contrôles Robot**: ZQSD + sliders vitesse
- 📷 **Contrôles Caméra**: OKLM (PTZ) + zoom + hauteur bras
- ⚙️ **Réglages**: Vitesse (0-100%), Zoom, Hauteur bras
- 🗺️ **Navigation**: Carte avec trajets sauvegardés
- 🖼️ **Galerie**: Vignettes photos/vidéos en direct

**Responsive**: Grille 3 colonnes × 9 lignes, adapte à 1920×1080 (Firefox Ubuntu)

### 2. **navig.html** - Navigation autonome

- 🗺️ Carte fullscreen (Saint-Cyr)
- 📍 Position du robot en temps réel
- 🎯 Définition de waypoints par clic
- 📊 Historique des trajets
- 📈 Affichage des chemins sauvegardés
- 🔄 Contrôles zoom/pan

### 3. **gallery.html** - Galerie complète

- 🖼️ Grille des photos/vidéos
- 🎬 Lecteur vidéo intégré
- 🔍 Fullscreen lightbox
- 🗑️ Suppression des fichiers
- 📥 Téléchargement des images
- 📊 Statistiques (nombre fichiers, poids total)

### 4. **terminal.html** - Logs système

- 📟 Terminal scrollable (défilement auto)
- 🎨 Coloration par type (info/warning/error)
- 📋 Copie-colle des logs
- 🔄 Auto-refresh
- 🔍 Filtrage par keywords
- 💾 Export CSV

### 5. **page_presentation.html** - Infos projet

- 📖 Présentation du projet AGRI-BOT
- 🎯 Mission et capacités
- 🏗️ Choix de conception
- 👥 Équipe ROB4
- 🔗 Lien GitHub

---

## 💾 Installation

### Prérequis

**Sur Zehno (Jussieu)**
- Ubuntu 22.04+ (ou Debian)
- ROS2 Jazzy
- Python 3.10+
- Navigateur Firefox/Chrome
- FFmpeg (pour WebRTC)

**Sur Raspberry Pi (Saint-Cyr)**
- Ubuntu Server 22.04 ARM64
- ROS2 Jazzy
- TurtleBot3 Burger packages
- Caméra Reolink avec flux RTSP

**Réseau**
- WiFi 2.4/5 GHz stable ou Tailscale VPN
- ROS_DOMAIN_ID=42 sur les deux machines
- ZENOH configuré pour le pont Jussieu ↔ Saint-Cyr (optionnel, peut utiliser DDS direct si même réseau)
- Tunnel socat pour RTSP (si caméra sur réseau distant)

### Étapes

#### 1. Cloner et compiler le projet

```bash
cd ~/ros2_ws/src
git clone <repo> web_control
cd ~/ros2_ws
colcon build --packages-select web_control --symlink-install
source install/setup.bash
```

#### 2. Installer les dépendances système (Zehno - Jussieu)

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-rosbridge-suite \
  python3-opencv \
  python3-pip \
  python3-venv \
  ffmpeg \
  socat
```

#### 3. Installer les dépendances WebRTC (environnement virtuel Python)

Le serveur WebRTC nécessite `aiortc` qui doit être installé dans un environnement virtuel :

```bash
cd ~/ros2_ws/src/web_control/web/stream/webrtc
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
deactivate
```

**Contenu de `requirements.txt`** :
```
aiortc==1.6.0
aiohttp==3.9.5
av
opencv-python
```

#### 4. Installer les dépendances (Raspberry - Saint-Cyr)

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-turtlebot3 \
  ros-jazzy-turtlebot3-bringup \
  ros-jazzy-rosbridge-suite
```

#### 5. Configurer le tunnel RTSP (si caméra sur réseau distant)

Si la caméra Reolink n'est pas sur le même réseau que le serveur WebRTC, créer un tunnel socat :

```bash
# Exemple de commande socat (à adapter selon votre configuration)
sudo socat TCP-LISTEN:8554,fork,reuseaddr TCP:<IP_CAMERA>:8554
```

Pour un lancement automatique au démarrage, créer un service systemd.

---

## ⚙️ Configuration

### 🔗 Connexion Réseau Multi-Site (Jussieu ↔ Saint-Cyr)

**Deux solutions sont en cours d'évaluation**:

#### Option 1: Tailscale VPN (Test en cours)

> **🚧 EN CONSTRUCTION** - Tailscale est documenté mais pas encore testé en production. Configuration en attente de validation.

**Tailscale** est un VPN mesh moderne, simple à configurer, qui crée un réseau privé virtuel entre les deux sites.

**Installation sur Zehno (Jussieu)**:
```bash
# Installer Tailscale
curl -fsSL https://tailscale.com/install.sh | sh

# Démarrer et s'authentifier
sudo tailscale up

# Vérifier l'IP Tailscale attribuée
tailscale ip -4
# Exemple: 100.64.x.x
```

**Installation sur Raspberry (Saint-Cyr)**:
```bash
# Même procédure
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
tailscale ip -4
```

**Configuration ROS2 avec Tailscale**:
```bash
# Remplacer les IP locales par les IP Tailscale dans app.js
# Exemple: ws://100.64.x.x:9090 au lieu de ws://192.168.0.132:9090
```

**Avantages**:
- ✅ Configuration simple (2 commandes)
- ✅ Pas besoin d'ouvrir des ports sur les firewalls
- ✅ Chiffrement automatique
- ✅ Fonctionne même avec NAT/Firewall complexes
- ✅ Gratuit pour usage personnel (<20 devices)

#### Option 2: VPN Polytech (Production)

> **🚧 EN CONSTRUCTION** - Configuration en attente de l'équipe informatique Polytech.

**VPN institutionnel** configuré et maintenu par l'équipe informatique de Polytech Sorbonne.

**Avantages**:
- ✅ Support technique assuré par Polytech
- ✅ Infrastructure déjà en place
- ✅ Sécurité validée par l'institution
- ✅ Intégration avec le réseau Polytech

**Configuration**: Se référer à la documentation fournie par l'équipe informatique Polytech.

> **💡 Recommandation**: Utiliser **Tailscale** pour le développement et les tests rapides, puis migrer vers le **VPN Polytech** pour la mise en production une fois configuré par l'équipe informatique.

---

### Middleware ROS2 - ZENOH

**Important**: Le projet utilise **rmw_zenoh_cpp** pour la communication multi-site.

```bash
# Sur les deux machines
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=42
```

Cela permet la découverte automatique et la communication entre Jussieu et Saint-Cyr via DDS/ZENOH.

### Caméra - image_tools

Le projet utilise **ros2 image_tools cam2image** pour capturer depuis une caméra USB:

```bash
# Sur le PC Zehno (capture locale)
ros2 run image_tools cam2image --ros-args -r image:=/image_raw -p width:=1280 -p height:=720 -p frequency:=30.0
```

**Paramètres**:
- `image:=/image_raw` - Nom du topic (doit matcher ce qu'attend web_video_server)
- `width:=1280` - Résolution (adapte selon ta caméra)
- `height:=720` - Résolution (adapte selon ta caméra)
- `frequency:=30.0` - FPS (30 FPS recommended)

Pour tester la caméra avant:
```bash
ls -la /dev/video*  # Vérifier que la caméra existe
v4l2-ctl -d /dev/video0 --all  # Lister les capacités
```

### Variables d'environnement

**Raspberry Pi** (`~/.bashrc`)
```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=42
export TURTLEBOT3_MODEL=burger
source /opt/ros/jazzy/setup.bash
```

**Zehno PC** (`~/.bashrc`)
```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
```

### IPs et adresses

Mettre à jour `src/web_control/web/app.js` ligne 5:

```javascript
const robotIp = "192.168.x.x";  // IP réelle de la Raspberry
```

### Répertoires de stockage

Créés automatiquement par `backend_node.py`:
- **Galerie**: `~/robot_gallery/` (photos/vidéos)
- **Trajets**: `~/trajectories/` (fichiers JSON)
- **Web files**: `<package_share>/web/` (HTML/CSS/JS)

---

## 🚀 Lancement

### Vue d'ensemble

Le système nécessite plusieurs composants :
1. **Serveur WebRTC** (port 8091) - Streaming vidéo bas latence depuis RTSP
2. **Backend ROS2** (port 8000) - Interface web + capture photo/vidéo + galerie
3. **Rosbridge** (port 9090) - Communication ROS2 ↔ WebSocket
4. **Robot TurtleBot3** (sur Raspberry) - Contrôle moteurs et capteurs
5. **Tunnel RTSP** (optionnel) - Si caméra sur réseau distant

### Lancement complet (mode production)

#### Sur Raspberry Pi (Saint-Cyr)

```bash
#!/bin/bash
# Fichier: ~/start_robot.sh

source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=42
export TURTLEBOT3_MODEL=burger

# Démarrer le robot TurtleBot3
ros2 launch turtlebot3_bringup robot.launch.py &
sleep 3

# Démarrer rosbridge pour communication WebSocket
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090
```

#### Sur Zehno (Jussieu)

**Terminal 1 - Serveur WebRTC (streaming vidéo)**
```bash
#!/bin/bash
# Fichier: ~/start_webrtc.sh

cd ~/ros2_ws/src/web_control/web/stream/webrtc

# Configurer l'URL RTSP de la caméra (si différente)
export RTSP_URL="rtsp://admin:ros2_2025@100.73.141.53:8554/h264Preview_01_main"
export RTSP_TRANSPORT="tcp"

# Lancer le serveur WebRTC
.venv/bin/python webrtc_server.py
```

**Terminal 2 - Backend ROS2 + Interface Web**
```bash
#!/bin/bash
# Fichier: ~/start_backend.sh

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=42

# Lancer le backend web (interface + galerie + uploads)
ros2 launch web_control web_control_full.launch.py
```

**Terminal 3 - Rosbridge local (optionnel si rosbridge sur Raspberry)**
```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=42

ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090
```

**Terminal 4 - Tunnel RTSP (si caméra sur réseau distant)**
```bash
# Exemple avec socat (à adapter selon votre configuration réseau)
sudo socat TCP-LISTEN:8554,fork,reuseaddr TCP:<IP_CAMERA_REELLE>:8554
```

### Lancement séparé (mode debug/développement)

Pour tester chaque composant individuellement :

**1. Test du serveur WebRTC seul**
```bash
cd ~/ros2_ws/src/web_control/web/stream/webrtc
.venv/bin/python webrtc_server.py

# Puis ouvrir http://localhost:8091 dans le navigateur
# (page de test WebRTC standalone)
```

**2. Test du backend seul**
```bash
cd ~/ros2_ws
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=42

python3 -m web_control.backend_node

# Puis ouvrir http://localhost:8000 dans le navigateur
```

**3. Test du flux RTSP avec ffplay**
```bash
# Vérifier que la caméra est accessible
ffplay -rtsp_transport tcp rtsp://admin:ros2_2025@100.73.141.53:8554/h264Preview_01_main

# Vérifier les détails du flux (codec, résolution, framerate)
ffprobe -rtsp_transport tcp rtsp://admin:ros2_2025@100.73.141.53:8554/h264Preview_01_main
```

**4. Test de rosbridge**
```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=42

ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090

# Vérifier dans le navigateur : ws://localhost:9090
```

### Vérification des services

Une fois tout lancé, vérifier que tous les ports sont ouverts :

```bash
# Vérifier les ports en écoute
netstat -tlnp | grep -E "8000|8091|9090"

# Devrait afficher :
# tcp  0  0  0.0.0.0:8000   LISTEN   (backend_node)
# tcp  0  0  0.0.0.0:8091   LISTEN   (webrtc_server.py)
# tcp  0  0  0.0.0.0:9090   LISTEN   (rosbridge)
```

Vérifier les topics ROS2 :

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=42

# Lister tous les topics
ros2 topic list

# Devrait inclure :
# /cmd_vel (commandes moteur)
# /battery_state (niveau batterie)
# /odom (odométrie)
# /gallery_files (liste fichiers galerie)
```

### Accéder à l'interface

Depuis n'importe quel navigateur sur le même réseau :

```
http://<IP_ZEHNO>:8000
```

Exemple: `http://192.168.0.150:8000`

---

## 🎮 Guide d'utilisation

### Configuration ZENOH pour multi-site

Si tu dois communiquer entre Jussieu et Saint-Cyr (sites différents):

1. **Démarre le daemon ZENOH** sur une machine centrale:
```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

2. **Configure les variables** sur toutes les autres machines:
```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=42
```

3. **Teste la découverte**:
```bash
ros2 node list      # Doit afficher les nodes des 2 sites
ros2 topic list     # Doit afficher /cmd_vel, /image_raw, etc.
```

### Vérifier la caméra

| Touche | Action | Page |
|--------|--------|------|
| **Z** | Avancer | index |
| **S** | Reculer | index |
| **Q** | Tourner gauche | index |
| **D** | Tourner droite | index |
| **O** | Caméra haut | index |
| **K** | Caméra gauche | index |
| **L** | Caméra bas | index |
| **M** | Caméra droite | index |

### Interface principale (index.html)

**Section vidéo (gauche)**
- 🎥 Flux live MJPEG de la caméra
- Clic pour plein écran

**Section contrôles (centre-bas)**
- 🕹️ Pavé directionnel ZQSD
- 📷 Contrôles PTZ OKLM
- 📸 Boutons Photo/REC
- 🎙️ Micro, 💡 Lampe

**Section réglages (droite)**
- 🚀 Vitesse robot (0-100%)
- 🔍 Zoom optique (0-100%)
- 🦾 Hauteur bras (30cm-1m20)
- 🛑 Arrêt d'urgence

**Section infos (bas-droite)**
- 🗺️ Lien vers Navigation
- 🖼️ Galerie miniatures
- 📟 Lien vers Terminal

### Navigation (navig.html)

- Clic sur la carte = créer waypoint
- Glisser = pan
- Zoom ±
- Charger trajet sauvegardé
- Lancer mission autonome

### Galerie (gallery.html)

- Grille complète des photos/vidéos
- Clic = fullscreen lightbox
- Bouton × = supprimer
- Télécharger via menu contextuel

### Terminal (terminal.html)

- Logs en temps réel
- Filtrer par type (INFO/WARN/ERROR)
- Copier les logs
- Export CSV

---

## 📡 API & Topics ROS2

### Topics publiés par app.js (Navigateur → Robot)

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | TwistStamped | Commandes moteurs (avancer/tourner) |
| `/camera/ptz` | Point | Pan/tilt caméra (x, y) |
| `/robot/arm_height` | Float32 | Hauteur bras pantographe (0-1.0) |
| `/camera/zoom` | Float32 | Zoom optique (0-100) |
| `/ui/save_trajectory` | String | Sauvegarde trajet (JSON path) |
| `/ui/delete_trajectory` | String | Supprime trajet |
| `/camera/delete_image` | String | Supprime photo/vidéo |
| `/robot/emergency_stop` | Bool | Arrêt d'urgence |
| `/ui/click` | Point | Coordonnées clic sur carte |

### Topics reçus par app.js (Robot/Backend → Navigateur)

| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| `/ui/trajectory_files` | String (JSON) | backend_node | `["jean.json", "matt_o.json", ...]` |
| `/ui/gallery_files` | String (JSON) | backend_node | `["photo1.jpg", "video1.mp4", ...]` |
| `/ui/system_logs` | String (JSON) | backend_node | `{"message": "...", "level": "info"}` |
| `/battery_state` | Float32 | TurtleBot | Pourcentage batterie (0-100) |
| `/odom` | Odometry | TurtleBot | Position/orientation (x, y, θ) |
| `/image_raw` | Image (RGB) | Caméra | Flux vidéo brut |

### Services

| Service | Type | Requête | Réponse |
|---------|------|---------|---------|
| `/camera/take_photo` | Trigger | - | success: bool |
| `/camera/start_video` | Trigger | - | success: bool |
| `/camera/stop_video` | Trigger | - | success: bool |

### Connexions WebSocket (app.js)

**app.js utilise 2 connexions ROSLIB.js parallèles**:

```javascript
// Connexion 1: Local (Jussieu) - Galerie/Trajets/Logs/Batterie
const ros = new ROSLIB.Ros({ 
    url: 'ws://localhost:9090'  // ou ws://<IP_ZEHNO>:9090
});

// Connexion 2: Direct robot (Saint-Cyr) - Commandes moteurs
const rosRobot = new ROSLIB.Ros({ 
    url: 'ws://192.168.x.x:9090'  // IP de la Raspberry
});
```

**Abonnements** (affichage):
- `trajListSub` → `/ui/trajectory_files`
- `gallerySub` → `/ui/gallery_files`
- `logSub` → `/ui/system_logs`
- `batterySub` → `/battery_state`

**Publications** (commandes):
- `cmdVelPub` (rosRobot) → `/cmd_vel`
- `ptzPub` (rosRobot) → `/camera/ptz`
- `saveTrajPub` (ros) → `/ui/save_trajectory`

---

## 🎯 Scénarios d'Utilisation Prévus

### Scénario 1: Inspection Matinale de Serre (Télé-opération)

**Objectif**: Vérifier l'état sanitaire des cultures depuis Jussieu

**Étapes**:
1. **Lancement** (6h30): Démarrage du robot, connexion WiFi
2. **Connexion web**: Ouverture de l'interface à Jussieu depuis navigateur
3. **Navigation manuelle**: Pilotage du robot entre les rangs de culture
4. **Capture visuelle**: Photos haute résolution de feuilles/fruits suspects
5. **Suivi capteurs**: Lecture température/humidité via pantographe variable
6. **Enregistrement**: Trajectoire mémorisée dans la galerie
7. **Fermeture**: Vidéos/images stockées, analyse hors-ligne si perte WiFi

**Durée**: ~45 min, autonomie nécessaire: 1-2h

**Fonctionnalités activées**: FP2 (télé-opération), FP3 (capture), FC3 (upload)

---

### Scénario 2: Patrouille Autonome Programmée

**Objectif**: Surveillance régulière sans intervention utilisateur

**Étapes**:
1. **Programmation**: Création d'une trajectoire via interface (waypoints sur carte)
2. **Conditions**: Déclenchement à horaire fixe ou sur alerte capteur
3. **Autonomie**: Robot suit le parcours sans contrôle utilisateur
4. **SLAM**: Adaptation automatique si obstacle imprévu
5. **Capture auto**: Photos tous les 2m le long du trajet
6. **Hors-ligne**: Données stockées localement en cas perte WiFi
7. **Retour**: Synchronisation des données au retour à base

**Durée**: Trajectoire 15-30 min selon complexité

**Fonctionnalités activées**: FP1 (parcours), FP4 (hors-ligne), FC2 (autonomie), FC4 (obstacles)

---

### Scénario 3: Alerte Urgente & Intervention

**Objectif**: Répondre à détection anomalie (maladie, ravageur, etc.)

**Étapes**:
1. **Capteur d'alerte**: Détecteur de chaleur/humidité anormal en serre
2. **Notification**: Alerte système reçue sur interface
3. **Navigation rapide**: Robot se déplace vers zone d'alerte automatiquement
4. **Inspection détaillée**: Zoom caméra, bras pantographe élévé
5. **Capture HD**: Images pour analyse expertises botanistes
6. **Rapide décision**: Données disponibles immédiatement pour décision traitement
7. **Traçabilité**: Géolocalisation + photo horodatées dans historique

**Durée**: Réaction < 30 sec, inspection 10-15 min

**Fonctionnalités activées**: FC2 (autonomie sur alerte), FP3 (caméra), FC4 (détection)

---

### Scénario 4: Monitoring Verger Extérieur

**Objectif**: Surveillance extensive du verger Saint-Cyr

**Étapes**:
1. **Lancement**: Robot sort par porte serre, accès verger
2. **Navigation GPS**: Utilisation GPS RTK pour positionnement ±5 cm
3. **Parcours exteriorVerger**: Longueur possible 100-200m via WiFi mesh
4. **Multi-zone**: Inspection rangées arbres fruitiers + zones ombragées
5. **Hors-ligne**: Si perte WiFi, robot continue SLAM 2D + mémorisation trajectoire
6. **Retour programmé**: Retrait auto à la base sur timeout batterie
7. **Synchronisation**: Upload données au retour

**Durée**: 2-3h selon taille zone

**Fonctionnalités activées**: FP4 (hors-ligne), FC10 (terrain changeant), FC11 (WiFi limitée)

---

### Scénario 5: Recherche & Débogage (Développeurs)

**Objectif**: Test nouvelles fonctionnalités de navigation/capteurs

**Étapes**:
1. **Mode développement**: Accès console terminal.html pour logs détaillés
2. **Test terrain**: Parcours test serre + verger avec enregistrement détaillé
3. **Post-analyse**: Export données, utilisation ROS bag pour rejouer mission
4. **Itération**: Modification paramètres, re-test (apprentissage machine possible)
5. **Documentation**: Capturer vidéo de la mission pour documentation

**Durée**: Flexible selon tests (30 min à plusieurs heures)

**Fonctionnalités activées**: Toutes, + accès logs système avancés

---

## 📊 Métriques de Performance Attendues

| Métrique | Cible | Actuel | Priorité |
|----------|-------|--------|----------|
| **Latence commande** | < 500 ms | 100-150 ms ✅ | F1 |
| **Résolution vidéo** | 480p min | 1280×720 ✅ | F1 |
| **Précision positionnement** (intérieur) | ±5 cm | À valider | F1 |
| **Franchissement marche** | 4 cm max | 15.5 cm ✅ | F1 |
| **Pente franchissable** | 5° max | À valider | F1 |
| **Autonomie batterie** | 3-4h | À valider | F1 |
| **Capacité stockage hors-ligne** | 1 Go | À valider | F3 |
| **Fiabilité détection obstacles** | >95% | À valider | F1 |

---

## 🔍 Troubleshooting

### Pas de vidéo (écran noir)

**Cause**: Serveur WebRTC non lancé ou flux RTSP inaccessible

**Solutions**:
```bash
# 1. Vérifier que le serveur WebRTC tourne
ps aux | grep webrtc_server
netstat -tlnp | grep 8091

# 2. Tester le flux RTSP directement
ffplay -rtsp_transport tcp rtsp://admin:ros2_2025@100.73.141.53:8554/h264Preview_01_main

# 3. Vérifier les logs du serveur WebRTC
cd ~/ros2_ws/src/web_control/web/stream/webrtc
.venv/bin/python webrtc_server.py
# Chercher : "WebRTC: client connecté" et "WebRTC: piste vidéo ajoutée"

# 4. Vérifier le tunnel socat (si utilisé)
ps aux | grep socat
sudo lsof -i :8554

# 5. Console navigateur (F12) - chercher erreurs WebSocket
# Doit voir : "WebRTC: connecté"
```

### Audio ne fonctionne pas

**Cause probable**: La caméra Reolink déclare une piste audio mais n'envoie pas de données

**Vérification**:
```bash
# 1. Tester l'audio avec ffplay
timeout 5 ffplay -nodisp -rtsp_transport tcp rtsp://admin:ros2_2025@100.73.141.53:8554/h264Preview_01_main

# 2. Vérifier que la piste audio existe
ffprobe -rtsp_transport tcp rtsp://admin:ros2_2025@100.73.141.53:8554/h264Preview_01_main 2>&1 | grep Audio

# 3. Activer l'audio dans l'interface web de la caméra
# Aller sur http://100.73.141.53 → Settings → Audio → Enable

# 4. Vérifier que le serveur WebRTC ajoute l'audio
# Dans les logs : "WebRTC: piste audio ajoutée"
```

**Note**: Si la caméra n'a pas de micro ou si l'audio est désactivé, les contrôles audio dans l'interface web ne fonctionneront pas. L'audio sera ajouté dans une future mise à jour une fois activé sur la caméra.

### Photo/Vidéo ne s'enregistre pas

**Cause**: Backend node non lancé ou permissions manquantes

**Solutions**:
```bash
# 1. Vérifier que backend_node tourne
ps aux | grep backend_node
netstat -tlnp | grep 8000

# 2. Vérifier les permissions du dossier galerie
ls -la ~/robot_gallery/
chmod 755 ~/robot_gallery/

# 3. Vérifier les logs backend_node
cd ~/ros2_ws
source install/setup.bash
python3 -m web_control.backend_node
# Chercher : "Photo enregistrée" ou "Vidéo enregistrée"

# 4. Tester l'upload manuellement
curl -X POST "http://localhost:8000/upload_photo?filename=test.jpg" \
  --data-binary "@/path/to/test.jpg"
```

### Navigateur affiche "Déconnecté" (ROS2)

**Cause**: rosbridge n'est pas accessible

**Solutions**:
```bash
# Vérifier que rosbridge tourne
ps aux | grep rosbridge

# Vérifier le port 9090
netstat -tuln | grep 9090

# Vérifier l'IP Zehno
hostname -I

# Tester la connexion depuis Firefox
http://<IP_ZEHNO>:8000

# Vérifier que ROS_DOMAIN_ID est identique
echo "Zehno: $ROS_DOMAIN_ID"
ssh pi@192.168.0.132 "echo Raspberry: \$ROS_DOMAIN_ID"
```

### Robot ne bouge pas malgré les contrôles

**Cause**: `/cmd_vel` n'atteint pas la Raspberry ou rosbridge ne marche pas

**Solutions**:
```bash
# Sur la Raspberry: écouter /cmd_vel
ros2 topic echo /cmd_vel

# Depuis Zehno: envoyer un test
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: "base_link"}, twist: {linear: {x: 0.2}}}'

# Vérifier la connexion rosbridge
ps aux | grep rosbridge

# Vérifier ROS_DOMAIN_ID (CRITIQUE avec ZENOH)
echo $ROS_DOMAIN_ID  # Doit être 42 sur les 2 machines

# Vérifier RMW_IMPLEMENTATION (ZENOH)
echo $RMW_IMPLEMENTATION  # Doit être rmw_zenoh_cpp
```

### Galerie vide

**Cause**: backend_node ne publie pas ou fichiers manquants

**Solutions**:
```bash
# Vérifier le topic
ros2 topic echo /ui/gallery_files

# Vérifier les fichiers
ls -la ~/robot_gallery/

# Vérifier le lien symbolique
ls -la ~/ros2_ws/src/web_control/web/gallery

# Vérifier que backend_node tourne
ps aux | grep backend_node

# Redémarrer backend_node
pkill -f backend_node
cd ~/ros2_ws && source install/setup.bash
ros2 run web_control backend_node
```

### Latence vidéo élevée

**Solutions WebRTC** (déjà faible latence):
```bash
# 1. Forcer UDP au lieu de TCP pour RTSP (plus rapide mais moins fiable)
export RTSP_TRANSPORT=udp
cd ~/ros2_ws/src/web_control/web/stream/webrtc
.venv/bin/python webrtc_server.py

# 2. Réduire la résolution de la caméra
# Dans l'interface web Reolink : Settings → Video → Resolution → 1280x720

# 3. Vérifier la bande passante réseau
ping 100.73.141.53  # Ping de la caméra
iperf3 -c <IP_RASPBERRY>  # Test débit

# 4. Utiliser Ethernet au lieu de WiFi si possible
```

### Connection perdue (ZENOH/DDS) - **CRITIQUE**

**Symptômes**: Tous les topics disparus, déconnexion complète même si les machines ping

**Causes possibles**:
1. ROS_DOMAIN_ID ou RMW_IMPLEMENTATION non alignés entre les deux sites
2. **VPN/Tailscale déconnecté** (si utilisé)
3. Firewall bloquant les ports ZENOH (7447)

**Solutions - Vérification Complète**:

```bash
# ÉTAPE 0: Vérifier connexion VPN (si Tailscale/VPN Polytech utilisé)
# Sur Zehno:
tailscale status  # Doit montrer "connected"
ping <IP_TAILSCALE_RASPBERRY>  # Ex: ping 100.64.x.x

# Sur Raspberry (SSH):
ssh pi@<IP_TAILSCALE>  # Utiliser l'IP Tailscale, pas l'IP locale
tailscale status

# ÉTAPE 1: Sur ZEHNO (Jussieu)
echo "=== Vérification Zehno ==="
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
ps aux | grep -E "rosbridge|backend|zenohd"
ros2 node list  # Doit afficher les nodes ROS2 locaux

# ÉTAPE 2: Sur RASPBERRY (Saint-Cyr)
echo "=== Vérification Raspberry (SSH) ==="
ssh pi@192.168.0.132 'bash -c "
echo ROS_DOMAIN_ID: \$ROS_DOMAIN_ID
echo RMW_IMPLEMENTATION: \$RMW_IMPLEMENTATION
ps aux | grep -E \"rosbridge|robot.launch|zenohd\"
ros2 node list
"'

# ÉTAPE 3: Redémarrer le daemon ZENOH (si utilisé)
# Sur Zehno:
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run rmw_zenoh_cpp rmw_zenohd &

# ÉTAPE 4: Vérifier la découverte DDS
ros2 node list  # Attendre 5 sec, doit inclure les nodes Raspberry

# ÉTAPE 5: Test ping des topics ZENOH
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 topic list
ros2 topic echo /cmd_vel  # Doit recevoir les messages de la Raspberry
```

**Checklist ZENOH**:
- [ ] `ROS_DOMAIN_ID=42` sur les 2 machines
- [ ] `RMW_IMPLEMENTATION=rmw_zenoh_cpp` export dans setup.bash ou .bashrc
- [ ] `rmw_zenohd` daemon lancé (optionnel mais recommandé pour multi-site)
- [ ] WiFi fonctionnelle entre les 2 sites (test: `ping 192.168.0.132`)
- [ ] Pare-feu n'existe pas entre Zehno et Raspberry (port 7447 ZENOH)
- [ ] Tous les nodes relancés APRÈS export des variables

### ZENOH - Configuration Avancée

**Fichier de configuration personnalisé** (optionnel):
```bash
# Créer config ZENOH personnalisée
cat > ~/zenoh.json <<EOF
{
  "session": {
    "router": {
      "links": ["tcp/0.0.0.0:7447"]
    }
  }
}
EOF

# Utiliser la config
export ZENOH_CONFIG=~/zenoh.json
ros2 run rmw_zenoh_cpp rmw_zenohd
```

**Diagnostic ZENOH détaillé**:
```bash
# Afficher tous les routers ZENOH actifs
export RUST_LOG=zenoh=debug
ros2 run rmw_zenoh_cpp rmw_zenohd  # Affiche les logs de connexion

# Depuis un autre terminal, tester la découverte
ros2 daemon stop
ros2 daemon start
ros2 node list  # Doit apparaître plus rapide avec daemon
```

### Tailscale VPN ne fonctionne pas

**Symptômes**: Impossible de ping les IP Tailscale, connexion refusée

**Solutions**:

```bash
# Vérifier status Tailscale
tailscale status
# Doit afficher: connected, liste des peers avec IPs

# Redémarrer Tailscale
sudo systemctl restart tailscaled
sudo tailscale up

# Vérifier les IP attribuées
tailscale ip -4
tailscale ip -6

# Tester la connectivité
ping <IP_TAILSCALE_AUTRE_MACHINE>

# Vérifier les routes
ip route | grep tailscale

# Logs Tailscale pour diagnostic
sudo journalctl -u tailscaled -n 50
```

**Si Tailscale se connecte mais ROS2 ne fonctionne pas**:
```bash
# Vérifier que app.js utilise les bonnes IP Tailscale
# Fichier: web/app.js ligne 15
const rosRobot = new ROSLIB.Ros({ 
    url: 'ws://100.64.x.x:9090'  // Remplacer par IP Tailscale, pas 192.168.x.x
});

# Vérifier firewall n'est pas actif
sudo ufw status  # Doit être inactive ou autoriser ports 9090, 8080, 7447
```

### Connection perdue (ZENOH/DDS)

**Vérifier la configuration ZENOH** sur Jussieu ↔ Saint-Cyr

```bash
# Configuration ROS2/DDS
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG=path/to/zenoh.json  # Si config externe
```

---

## 📁 Structure du projet

```
ros2_ws/
├── src/
│   └── web_control/
│       ├── package.xml                    # Config ROS2
│       ├── setup.py                       # Installation Python
│       ├── setup.cfg
│       │
│       ├── web_control/                   # Package Python
│       │   ├── __init__.py
│       │   ├── backend_node.py            # Node ROS2 principal (333 lignes)
│       │   │   ├─ Serveur HTTP (port 8000)
│       │   │   ├─ Publishers: /ui/trajectory_files, /ui/gallery_files
│       │   │   ├─ Subscribers: /ui/save_trajectory, /camera/delete_image
│       │   │   └─ Managers: Capture & Gallery
│       │   ├── capture_manager.py         # Gestion capture photos/vidéos
│       │   └── gallery_manager.py         # Gestion fichiers galerie
│       │
│       ├── launch/
│       │   └── web_control_full.launch.py # Lance: rosbridge, web_video_server, backend
│       │
│       └── web/                           # Fichiers web statiques (servis par HTTP)
│           ├── index.html                 # 🎮 Page principale (contrôles)
│           ├── app.js                     # Logique JS (672 lignes)
│           │   ├─ 2 connexions ROSLIB.js
│           │   ├─ Évenements clavier (Z/Q/S/D, O/K/L/M)
│           │   ├─ Logging complet
│           │   └─ Gestion galerie/trajets
│           ├── style.css                  # Styles responsive (vh units)
│           │   ├─ Grille 3×9 (1920×1080)
│           │   ├─ Mode sombre/clair
│           │   └─ Gradients, animations
│           ├── logo-polytech.png
│           ├── saint-cyr.png              # Fond carte navigation
│           │
│           ├── galerie/                   # 📸 Page galerie
│           │   └── gallery.html           # Grille complète photos/vidéos
│           │
│           ├── terminal/                  # 📟 Page terminal
│           │   ├── terminal.html
│           │   └── terminal.js            # Logs scrollables
│           │
│           ├── navig/                     # 🗺️ Page navigation
│           │   ├── navig.html
│           │   └── navig.js               # Carte interactive
│           │
│           ├── accueil/                   # 📖 Page présentation
│           │   ├── page_presentation.html # Infos projet AGRI-BOT
│           │   ├── home.png
│           │   ├── bras.jpg               # Photos du robot
│           │   └── CURT.jpg
│
├── install/
│   └── web_control/
│       ├── lib/python3.12/site-packages/web_control/  # Code compilé
│       └── share/web_control/                         # Assets statiques
│           └── web/                                   # Fichiers web
│               └── gallery/                           # 📁 Photos/vidéos capturées
│                   └── (fichiers générés par capture_manager)
│
├── build/                                 # Fichiers de compilation
└── log/                                   # Logs colcon
```

### Fichiers clés détaillés

#### `web_control/backend_node.py` (333 lignes)
- Node ROS2 principal
- Lance serveur HTTP sur port 8000
- Publie listes périodiquement:
  - `/ui/trajectory_files` (tous les 2s)
  - `/ui/gallery_files` (avec debouncing)
  - `/ui/system_logs` (événements)
- Écoute:
  - `/ui/save_trajectory` → sauvegarde JSON
  - `/ui/delete_trajectory` → suppression
  - `/camera/delete_image` → suppression galerie
  - `/robot/emergency_stop` → arrêt urgence
- Gestion des répertoires `~/robot_gallery/` et `~/trajectories/`

#### `web_control/capture_manager.py`
- Capture photos/vidéos depuis `/image_raw`
- Sauvegarde dans `~/robot_gallery/`
- Format: JPEG pour photos, MP4 pour vidéos
- Timestamps automatiques dans les noms

#### `web_control/gallery_manager.py`
- Scanne `~/robot_gallery/` et `~/trajectories/`
- Liste fichiers avec métadonnées
- Gère suppression/archivage
- Export JSON pour le frontend

#### `web/app.js` (672 lignes)
**Connexions** (lignes 1-20)
- WebSocket 1: `ros` → localhost:9090 (galerie/trajets/logs)
- WebSocket 2: `rosRobot` → 192.168.x.x:9090 (commandes moteurs)

**Subscribers** (lignes 85-135)
- `/ui/trajectory_files` → `updateTrajectoryList()`
- `/ui/gallery_files` → `updateGallery()`
- `/ui/system_logs` → terminal display
- `/battery_state` → affichage batterie

**Publishers** (lignes 35-75)
- `/cmd_vel` → `sendCmd()`
- `/camera/ptz` → `sendPtz()`
- `/ui/save_trajectory` → trajectoires
- `/robot/emergency_stop` → arrêt urgence

**Événements** (lignes 150-450)
- Clavier: Z/Q/S/D, O/K/L/M
- Souris: clic sur carte, boutons
- Sliders: vitesse, zoom, hauteur bras

**UI Updates** (lignes 450-672)
- `updateGallery()`: Grille avec suppressions
- `updateTrajectoryList()`: Sélecteur trajets
- `updateSpeed()`, `updateZoom()`, `updateArm()`: Sliders
- Logging système complet

#### `web/index.html`
- Structure HTML5 sémantique
- Sections: vidéo | contrôles | réglages | carte | galerie
- Modal pour paramètres avancés
- Navbar avec liens internes

#### `web/style.css`
- **Unités**: 100% `vh` (viewport height)
- **Grille**: 3 colonnes × 9 lignes (1920×1080)
- **Responsive**: Media queries pour mobile
- **Thème**: Mode sombre/clair avec CSS variables
- **Animations**: Transitions, hover effects
- **Gradients**: Séparateurs, boutons

#### `web/navig.html`
- Canvas fullscreen pour la carte
- Zoom/pan avec molette/drag
- Waypoints par clic
- Historique trajets

#### `web/terminal.html`
- Affichage scrollable des logs
- Coloration (INFO/WARN/ERROR)
- Filtrage par keywords
- Export CSV

#### `web/galerie/gallery.html`
- Grille CSS `grid` (auto-fit)
- Lightbox fullscreen
- Suppression avec confirmation
- Statistiques fichiers
- Affiche les fichiers depuis `install/web_control/share/web_control/web/gallery/`

#### `web/page_presentation.html`
- 📖 Infos projet AGRI-BOT
- 🎯 Mission, capacités, conception
- 👥 Équipe ROB4
- 🔗 Lien GitHub

#### `launch/web_control_full.launch.py`
```python
# Lance 3 nodes:
1. rosbridge_websocket (port 9090)
2. web_video_server (port 8080)
3. backend_node (web server port 8000)
```

---

## 🔧 Configuration avancée

### Changer la qualité vidéo

Fichier: `web/app.js` ligne 9
```javascript
// Paramètres: quality (0-100), width, height, fps
videoElement.src = `http://${videoHost}:8080/stream?topic=/image_raw&type=mjpeg&quality=80&width=1280&height=720`;
```

### Ajuster vitesses max

Fichier: `web/app.js` fonction `sendCmd()` (ligne ~173)
```javascript
msg.twist.linear.x = 0.5 * robotSpeed;      // Changer 0.5 (m/s)
msg.twist.angular.z = 2.84 * angularSpeed;  // Changer 2.84 (rad/s)
```

### Ajouter un nouveau topic

**Dans app.js**:
```javascript
const monTopic = new ROSLIB.Topic({
    ros: ros,  // ou rosRobot
    name: '/mon/topic',
    messageType: 'std_msgs/Float32'
});
monTopic.subscribe((msg) => {
    console.log("Reçu:", msg.data);
});
```

**Dans backend_node.py**:
```python
self.mon_pub = self.create_publisher(Float32, '/mon/topic', 10)
self.mon_pub.publish(Float32(data=valeur))
```

### Modes opérationnels

1. **Mode Direct**: Contrôle via l'interface web (actuel)
2. **Mode Autonome**: Navigation préprogrammée (trajets)
3. **Mode Hors-Ligne**: Enregistrement local des données

---

## 📊 Spécifications de performances

| Métrique | Valeur |
|----------|--------|
| **Latence vidéo** | 200-500ms (MJPEG) |
| **Latence commandes** | 50-100ms (WebSocket) |
| **Bande vidéo** | ~500KB/s @ 10 FPS + quality 100 |
| **Vitesse linéaire max** | 0.5 m/s |
| **Vitesse angulaire max** | 2.84 rad/s (plafonné 93%) |
| **Autonomie estimée** | 4-6 heures (dépend batterie TurtleBot) |
| **Portée WiFi** | ~50-100m en ligne directe |

---

## 🚀 Démarrage Rapide (Checklist)

### ✅ Avant de Lancer (Vérifications Système)

**Sur ZEHNO (Jussieu)**:
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Vérifier ROS2
ros2 --version
which colcon

# Vérifier base de code
ls ~/ros2_ws/src/web_control/
ls ~/ros2_ws/install/
```

**Sur RASPBERRY (Saint-Cyr)**:
```bash
ssh pi@192.168.0.132
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Vérifier connexion réseau
ping 192.168.0.1  # Gateway WiFi
ping 192.168.1.XX  # IP Zehno
```

### 🎯 Lancement Optimal (Ordre Important!)

**Terminal 1 - ZEHNO: Daemon ZENOH** (optionnel mais recommandé):
```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=42
ros2 run rmw_zenoh_cpp rmw_zenohd
# Affiche: [INFO] ... Ready!
```

**Terminal 2 - RASPBERRY: Robot Launch** (SSH):
```bash
ssh pi@192.168.0.132
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# ⚠️  TurtleBot3 temporaire (en attendant Curt Mini)
ros2 launch turtlebot3_bringup robot.launch.py

# 📌 Après réception Curt Mini, remplacer par:
# ros2 launch curt_mini_bringup robot.launch.py

# Attendre 10 secondes, vérifier: [INFO] model_state ... Ready!
```

**Terminal 3 - RASPBERRY: Rosbridge** (SSH, nouvel onglet):
```bash
ssh pi@192.168.0.132
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run rosbridge_server rosbridge_websocket
# Affiche: Rosbridge WebSocket server started on port 9090
```

**Terminal 4 - ZEHNO: Caméra** (si disponible):
```bash
cd ~/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run image_tools cam2image --ros-args -r image:=/image_raw -p width:=1280 -p height:=720 -p frequency:=30.0
# Affiche: Publishing camera images...
```

**Terminal 5 - ZEHNO: Backend & Web** (SSH):
```bash
cd ~/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 launch web_control web_control_full.launch.py
# Affiche: [INFO] backend_node ... HTTP server on port 8000
```

**Terminal 6 - ZEHNO: Rosbridge** (SSH, nouvel onglet):
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run rosbridge_server rosbridge_websocket
# Affiche: Rosbridge WebSocket server started on port 9090
```

**Navigateur**: Ouvrir dans Firefox/Chrome:
```
http://localhost:8000
```

### ⚡ Vérification Rapide

```bash
# Dans un nouveau terminal Zehno:
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Les nodes de Raspberry doivent apparaître!
ros2 node list | grep -E "robot|rosbridge"

# Les topics ZENOH doivent être visibles
ros2 topic list | grep -E "cmd_vel|image_raw|battery"

# Test rapide: Envoyer une commande
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: "base_link"}, twist: {linear: {x: 0.1}}}'

# Vérifier réception sur Raspberry
ssh pi@192.168.0.132 'ros2 topic echo /cmd_vel --once'
```

### ❌ Si ça ne marche pas

**Symptôme**: "Déconnecté" dans le navigateur ou topics vides

**Diagnostic rapide**:
```bash
# 1. Vérifier domaine et implémentation
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID, RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"

# 2. Vérifier rosbridge sur les 2 sites
ps aux | grep rosbridge_websocket

# 3. Vérifier connectivité réseau
ping 192.168.0.132

# 4. Relancer tous les services
pkill -f rosbridge_websocket
pkill -f backend_node
# Puis relancer terminaux dans l'ordre ci-dessus
```

→ Consulter [Troubleshooting - Connection perdue (ZENOH/DDS)](#connection-perdue-zenohds---critique) pour diagnostic approfondi

---

## 📝 Notes de développement

### 🚧 État Actuel du Projet (Février 2026)

**Plateforme de simulation actuelle**:
- ✅ **TurtleBot3 Burger** - Simule le comportement du Curt Mini
  - Permet de développer l'interface de contrôle
  - Valide l'architecture ROS2 + ZENOH
  - Teste la communication multi-site
- ✅ Raspberry Pi 4 avec ROS2 Jazzy
- ✅ PC Zehno (Jussieu) - Serveur web et backend
- ✅ Caméra USB V4L2 (temporaire, remplacée par PTZ)
- ✅ Réseau Tailscale VPN (test) ou VPN Polytech (production)

**🎯 Robot final AGRI-BOT (en cours d'acquisition)**:
- ⏳ **Curt Mini 4×4** (commande en cours, livraison attendue)
  - Base mobile IP68 étanche
  - 15.5 cm garde au sol, châssis à bascule
  - Charge utile 30 kg
  - Autonomie 3-4 heures
- ⏳ **Bras pantographe** (30 cm → 1m20) - Conception mécanique à finaliser
- ⏳ **Caméra PTZ professionnelle** - Montée sur le bras
- ⏳ **LiDAR 2D** - Pour SLAM et évitement d'obstacles
- ⏳ **GPS RTK** - Navigation extérieure précise (±5 cm)
- ⏳ **Éclairage LED** - Inspection en faible luminosité

**Logiciel développé et testé**:
- ✅ Interface web complète (HTML/CSS/JS) - 5 pages
- ✅ Backend ROS2 (backend_node.py, capture_manager, gallery_manager)
- ✅ Architecture ZENOH multi-site (Jussieu ↔ Saint-Cyr)
- ✅ Streaming vidéo MJPEG (via web_video_server)
- ✅ Système de galerie photos/vidéos
- ✅ Système de trajectoires sauvegardables
- ✅ Logging événements temps réel
- ✅ Contrôles clavier (ZQSD/OKLM) avec feedback visuel

**Phase actuelle**: 
- 🔄 **Développement et validation sur TurtleBot3** (plateforme de simulation)
- 🎯 **Migration vers Curt Mini planifiée** dès réception du matériel
- 🔧 **Intégration équipements finaux** (bras, LiDAR, PTZ) après livraison Curt Mini

**Timeline estimée**:
1. **Maintenant**: Tests et développement sur TurtleBot3
2. **Réception Curt Mini**: Migration du code, calibrage vitesses
3. **Phase 2**: Intégration bras pantographe + caméra PTZ
4. **Phase 3**: Intégration LiDAR + GPS RTK
5. **Phase 4**: Tests terrain en serre et verger Saint-Cyr

### Conventions de code

- **Topics UI**: `/ui/*` (String JSON)
- **Topics robot**: `/robot/*` (commandes)
- **Topics caméra**: `/camera/*` (PTZ, capture)
- **Topics navigation**: `/scan`, `/map`, `/gps/fix`
- **Topics batterie/odométrie**: From `robot.launch.py`

### Debugging

**Console navigateur (F12)**:
```javascript
console.log("Debug message");
logEvent("Événement système", "info");  // Affiche aussi dans terminal.html
```

**Écouter topics en temps réel**:
```bash
ros2 topic echo /mon/topic
ros2 topic pub --once /mon/topic std_msgs/msg/Float32 "{data: 1.5}"
```

**Vérifier connections ROS2**:
```bash
ros2 node list
ros2 topic list
ros2 service list
```

---

## 📄 Licence

À définir (voir `package.xml`)

---

## 👥 Équipe & Contacts

**Équipe ROB4 - Polytech Sorbonne**
- Fatoumata DIALLO
- Pierre-Louis PASUTTO
- Matthieu VINET

Encadrés par: **Aline BAUDRY**

**Dépôt GitHub**: [github.com/SHuttooo/ros2_ws](https://github.com/SHuttooo/ros2_ws)

---

## 📞 Support & Maintenance

### Problèmes courants

Consulter la section [Troubleshooting](#troubleshooting) pour:
- Connexion WebSocket échouée
- Caméra ne s'affiche pas
- Robot qui ne répond pas
- Galerie/Trajets vides
- Latence élevée
- Perte de connexion ZENOH/DDS

### Logs système

```bash
# Logs colcon build
cat ~/ros2_ws/log/latest_build/web_control/stdout.log

# Logs rosbridge
ros2 run rosbridge_server rosbridge_websocket  # Affiche en direct

# Logs backend_node
ros2 run web_control backend_node  # Affiche en direct
```

---

**Dernière mise à jour**: Février 2026  
**Version**: 1.0.0  
**ROS2 Version**: Jazzy  
**Python**: 3.10+

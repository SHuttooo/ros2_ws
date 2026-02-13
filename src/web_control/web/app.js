// =======================================================================
// 1. CONFIGURATION & CONNEXION
// =======================================================================
const serverIp = window.location.hostname;  // IP du serveur web
const robotIp = "192.168.0.132";            // IP du robot/Raspberry
const videoHost = serverIp === "" ? "localhost" : serverIp;

// Vidéo - Flux RTSP converti en MJPEG par FFmpeg (basse latence)
const videoElement = document.getElementById('cameraFeed');
videoElement.src = 'http://localhost:8090';

// WebSocket 1: Se connecte au serveur local (Zehno) pour galerie/trajets/logs
const ros = new ROSLIB.Ros({ url: `ws://${window.location.hostname}:9090` });

// WebSocket 2: Se connecte directement à la Raspberry pour /cmd_vel (commandes robot)
const rosRobot = new ROSLIB.Ros({ url: `ws://${robotIp}:9090` });

ros.on('connection', () => {
    document.getElementById('status').innerText = 'Connecté';
    document.getElementById('status').className = 'connected';
});
ros.on('error', (error) => {
    document.getElementById('status').innerText = 'Erreur';
    document.getElementById('status').className = 'disconnected';
});
ros.on('close', () => {
    document.getElementById('status').innerText = 'Déconnecté';
    document.getElementById('status').className = 'disconnected';
});

// =======================================================================
// 2. TOPICS
// =======================================================================

// Robot (TwistStamped - type imposé par un autre node)
const cmdVelPub = new ROSLIB.Topic({
    ros: rosRobot,  // Utilise la connexion directe à la Raspberry
    name: '/cmd_vel',
    messageType: 'geometry_msgs/TwistStamped'
});

// PTZ (Point: x, y)
const ptzPub = new ROSLIB.Topic({
    ros: ros,
    name: '/camera/ptz',
    messageType: 'geometry_msgs/Point' // Changé de Twist à Point
});

// Mission Status
const missionPub = new ROSLIB.Topic({
    ros: ros,
    name: '/mission/status',
    messageType: 'std_msgs/Bool'
});

// Delete Image (Nouveau)
const deletePub = new ROSLIB.Topic({
    ros: ros,
    name: '/camera/delete_image',
    messageType: 'std_msgs/String'
});

const zoomPub = new ROSLIB.Topic({ ros: ros, name: '/camera/zoom', messageType: 'std_msgs/Float32' });
const armPub = new ROSLIB.Topic({ ros: ros, name: '/robot/arm_height', messageType: 'std_msgs/Float32' });
const clickPub = new ROSLIB.Topic({ ros: ros, name: '/ui/click', messageType: 'geometry_msgs/Point' });

// Logs UI
const logPub = new ROSLIB.Topic({
    ros: ros,
    name: '/ui/system_logs',
    messageType: 'std_msgs/String'
});

function logEvent(message, level = 'info') {
    try {
        logPub.publish(new ROSLIB.Message({
            data: JSON.stringify({ message, level })
        }));
    } catch (e) {
        console.log(message);
    }
}

// Topic pour l'arrêt d'urgence
const emergencyPub = new ROSLIB.Topic({
    ros: ros,
    name: '/robot/emergency_stop',
    messageType: 'std_msgs/Bool'
});

// Topic pour recevoir la liste des fichiers de trajectoire
const trajListSub = new ROSLIB.Topic({ 
    ros: ros, 
    name: '/ui/trajectory_files', 
    messageType: 'std_msgs/String' 
});

trajListSub.subscribe((msg) => {
    try {
        updateTrajectoryList(JSON.parse(msg.data));
    } catch (e) {
        console.error("Erreur parsing liste trajets", e);
    }
});

const gallerySub = new ROSLIB.Topic({ ros: ros, name: '/ui/gallery_files', messageType: 'std_msgs/String' });
let lastGalleryData = "";
gallerySub.subscribe((msg) => { 
    try { 
        console.log("Galerie reçue:", msg.data);  // DEBUG
        if (msg.data !== lastGalleryData) {
            lastGalleryData = msg.data;
            updateGallery(JSON.parse(msg.data)); 
        }
    } catch (e) { 
        console.error("Erreur galerie:", e);  // DEBUG
    } 
});

// Topic pour recevoir le niveau de batterie
const batterySub = new ROSLIB.Topic({ 
    ros: ros, 
    name: '/robot/battery', 
    messageType: 'std_msgs/Float32' 
});

batterySub.subscribe((msg) => {
    const batteryLevel = Math.round(msg.data);
    const batteryElement = document.getElementById('battery');
    batteryElement.innerText = `🔋 ${batteryLevel}%`;
    
    // Changer la couleur selon le niveau
    batteryElement.classList.remove('battery-low', 'battery-critical');
    if (batteryLevel <= 20) {
        batteryElement.classList.add('battery-critical');
    } else if (batteryLevel <= 50) {
        batteryElement.classList.add('battery-low');
    }
});

// Services
const photoClient = new ROSLIB.Service({ ros: ros, name: '/camera/take_photo', serviceType: 'std_srvs/Trigger' });
const startVideoClient = new ROSLIB.Service({ ros: ros, name: '/camera/start_video', serviceType: 'std_srvs/Trigger' });
const stopVideoClient = new ROSLIB.Service({ ros: ros, name: '/camera/stop_video', serviceType: 'std_srvs/Trigger' });

// =======================================================================
// 3. LOGIQUE DE CONTRÔLE
// =======================================================================

let robotSpeed = 0.5;

function updateSpeed(val) {
    robotSpeed = parseInt(val) / 100.0;
    document.getElementById('speedVal').innerText = val + '%';
    localStorage.setItem('robotSpeed', val);
    logEvent(`Vitesse robot: ${val}%`, 'info');
}

// --- ROBOT (ZQSD) ---
function sendCmd(direction) {
    // 🛑 Arrêt automatique de la mission si on bouge le robot manuellement
    if (missionActive && direction !== 'stop') {
        toggleMission();
        console.log("Mission interrompue par mouvement manuel.");
    }

    // TwistStamped: structure avec header + twist
    let msg = new ROSLIB.Message({
        header: {
            stamp: { sec: 0, nanosec: 0 },
            frame_id: 'base_link'
        },
        twist: {
            linear: { x: 0.0, y: 0.0, z: 0.0 },
            angular: { x: 0.0, y: 0.0, z: 0.0 }
        }
    });

    if (direction === 'up')    msg.twist.linear.x = 0.19 * robotSpeed;
    if (direction === 'down')  msg.twist.linear.x = -0.19 * robotSpeed;
    if (direction === 'left')  msg.twist.angular.z = 2.80 * 0.93 *  robotSpeed;
    if (direction === 'right') msg.twist.angular.z = -2.80 * 0.93 * robotSpeed;

    cmdVelPub.publish(msg);
    logEvent(`Robot: ${direction}`, 'info');
}

// --- PTZ (OKLM) ---
function sendPtz(direction) {
    // Utilisation de Point (x, y)
    let point = new ROSLIB.Message({
        x: 0.0,
        y: 0.0,
        z: 0.0
    });

    if (direction === 'up')    point.x = 1.0;
    if (direction === 'down')  point.x = -1.0;
    if (direction === 'right') point.y = 1.0;
    if (direction === 'left')  point.y = -1.0;

    ptzPub.publish(point);
    logEvent(`Caméra PTZ: ${direction}`, 'info');
}

// --- CLAVIER ---
const keyState = {};
const keyToButton = {};

document.querySelectorAll('.dpad button').forEach((button) => {
    const hint = button.querySelector('.key-hint');
    if (!hint) return;
    const key = hint.textContent.trim().toLowerCase();
    if (key) keyToButton[key] = button;
});

document.addEventListener('keydown', (event) => {
    const key = event.key.toLowerCase();
    if (keyState[key]) return;
    keyState[key] = true;

    if (keyToButton[key]) keyToButton[key].classList.add('active-key');
    if (['z','q','s','d','o','k','l','m'].includes(key)) logEvent(`Clavier: ${key.toUpperCase()}`, 'info');

    if (key === 'z') sendCmd('up');
    if (key === 's') sendCmd('down');
    if (key === 'q') sendCmd('left');
    if (key === 'd') sendCmd('right');

    if (key === 'o') sendPtz('up');
    if (key === 'l') sendPtz('down');
    if (key === 'k') sendPtz('left');
    if (key === 'm') sendPtz('right');
});

document.addEventListener('keyup', (event) => {
    const key = event.key.toLowerCase();
    keyState[key] = false;

    if (keyToButton[key]) keyToButton[key].classList.remove('active-key');

    if (['z','q','s','d'].includes(key)) sendCmd('stop');
    if (['o','k','l','m'].includes(key)) sendPtz('stop');
});

// =======================================================================
// 4. AUTRES FONCTIONS
// =======================================================================

function updateZoom(val) {
    const elem = document.getElementById('zoomVal');
    const elemModal = document.getElementById('zoomValModal');
    if (elem) elem.innerText = val + '%';
    if (elemModal) elemModal.innerText = val + '%';
    localStorage.setItem('zoomValue', val);
    zoomPub.publish(new ROSLIB.Message({ data: parseFloat(val) }));
    logEvent(`Zoom optique: ${val}%`, 'info');
}

function updateArm(val) {
    const elem = document.getElementById('armVal');
    const elemModal = document.getElementById('armValModal');
    if (elem) elem.innerText = val + '%';
    if (elemModal) elemModal.innerText = val + '%';
    localStorage.setItem('armHeight', val);
    armPub.publish(new ROSLIB.Message({ data: parseFloat(val) }));
    logEvent(`Hauteur bras: ${val}%`, 'info');
}

// Gestion Lampe et Micro
let lampActive = false;
let micActive = false;

function toggleLamp() {
    const btn = document.getElementById('btnLamp');
    lampActive = !lampActive;
    if (lampActive) {
        btn.classList.add('active');
        logEvent('Lampe: activée', 'success');
    } else {
        btn.classList.remove('active');
        logEvent('Lampe: désactivée', 'warn');
    }
}

function toggleMic() {
    const btn = document.getElementById('btnMic');
    micActive = !micActive;
    if (micActive) {
        btn.classList.add('active');
        logEvent('Micro: activé', 'success');
    } else {
        btn.classList.remove('active');
        logEvent('Micro: désactivé', 'warn');
    }
}

// Arrêt d'urgence
function emergencyStop() {
    // Publie l'arrêt d'urgence vers ROS2
    emergencyPub.publish(new ROSLIB.Message({ data: true }));
    
    // Arrête tout mouvement du robot
    sendCmd('stop');
    sendPtz('stop');
    
    // Arrête la mission si active
    if (missionActive) {
        toggleMission();
    }
    
    // Arrête l'enregistrement vidéo si actif
    if (isRecording) {
        toggleVideo();
    }
    
    // Affiche une alerte visuelle
    const btn = document.getElementById('btnEmergency');
    btn.style.animation = 'pulse 0.5s 3';
    setTimeout(() => {
        btn.style.animation = '';
    }, 1500);
    
    console.log('ARRÊT D\'URGENCE ACTIVÉ');
    logEvent('ARRÊT D\'URGENCE ACTIVÉ', 'error');
}

// Gestion Mission
let missionActive = false;
function toggleMission() {
    const btn = document.getElementById('btnMission');
    missionActive = !missionActive;

    if (missionActive) {
        btn.innerText = "🛑 Arrêter la mission";
        btn.className = "mission-btn stop";
        missionPub.publish(new ROSLIB.Message({ data: true }));
        logEvent('Mission lancée', 'success');
    } else {
        btn.innerText = "🚀 Lancer la mission";
        btn.className = "mission-btn start";
        missionPub.publish(new ROSLIB.Message({ data: false }));
        logEvent('Mission arrêtée', 'warn');
    }
}

// Photo & Vidéo
let isRecording = false;
function takePhoto() {
    logEvent('Photo demandée', 'info');
    photoClient.callService(new ROSLIB.ServiceRequest(), (res) => {
        alert(res.success ? "📸 Prise !" : "Erreur");
        logEvent(res.success ? 'Photo prise' : 'Erreur prise photo', res.success ? 'success' : 'error');
    });
}
function toggleVideo() {
    let btn = document.getElementById('btnRecord');
    if (!isRecording) {
        logEvent('Démarrage enregistrement vidéo', 'info');
        startVideoClient.callService(new ROSLIB.ServiceRequest(), (res) => {
            if(res.success) { isRecording = true; btn.innerText = "⏹ STOP"; btn.style.backgroundColor = "black"; }
            logEvent(res.success ? 'Enregistrement vidéo démarré' : 'Erreur démarrage vidéo', res.success ? 'success' : 'error');
        });
    } else {
        logEvent('Arrêt enregistrement vidéo', 'info');
        stopVideoClient.callService(new ROSLIB.ServiceRequest(), (res) => {
            if(res.success) { isRecording = false; btn.innerText = "🔴 REC"; btn.style.backgroundColor = "#e74c3c"; }
            logEvent(res.success ? 'Enregistrement vidéo arrêté' : 'Erreur arrêt vidéo', res.success ? 'success' : 'error');
        });
    }
}

// Galerie & Suppression
function updateGallery(files) {
    const grid = document.getElementById('galleryGrid');
    grid.innerHTML = "";
    logEvent(`Galerie mise à jour (${files.length} fichiers)`, 'info');
    files.forEach(file => {
        // Conteneur item
        let div = document.createElement('div');
        div.className = "gallery-item";

        // Vérifier si c'est une vidéo
        const isVideo = file.toLowerCase().endsWith('.mp4') || file.toLowerCase().endsWith('.avi') || file.toLowerCase().endsWith('.mov');

        if (isVideo) {
            // Créer un élément vidéo avec miniature (même style que gallery.html)
            let video = document.createElement('video');
            video.src = 'gallery/' + file;
            video.controls = false;
            video.muted = true;
            video.preload = "metadata";
            video.playsInline = true;
            video.style.width = "100%";
            video.style.height = "100%";
            video.style.objectFit = "cover";
            video.onclick = () => {
                logEvent('Ouverture galerie (vidéo)', 'info');
                window.location.href = 'galerie/gallery.html';
            }; // Rediriger vers gallery

            // Fallback poster (triangle play)
            if (!window.__videoFallbackPoster) {
                const c = document.createElement('canvas');
                c.width = 320; c.height = 180;
                const cx = c.getContext('2d');
                cx.fillStyle = '#0f1419';
                cx.fillRect(0, 0, c.width, c.height);
                cx.fillStyle = '#4da3d8';
                cx.beginPath();
                cx.moveTo(c.width * 0.4, c.height * 0.3);
                cx.lineTo(c.width * 0.7, c.height * 0.5);
                cx.lineTo(c.width * 0.4, c.height * 0.7);
                cx.closePath();
                cx.fill();
                window.__videoFallbackPoster = c.toDataURL('image/png');
            }
            video.setAttribute('poster', window.__videoFallbackPoster);

            // Générer miniature depuis première frame
            const buildPoster = () => {
                try {
                    const canvas = document.createElement('canvas');
                    canvas.width = video.videoWidth || 320;
                    canvas.height = video.videoHeight || 180;
                    const ctx = canvas.getContext('2d');
                    ctx.drawImage(video, 0, 0, canvas.width, canvas.height);
                    video.setAttribute('poster', canvas.toDataURL('image/jpeg'));
                    video.pause();
                } catch (err) {
                    video.setAttribute('poster', window.__videoFallbackPoster);
                }
            };

            video.addEventListener('loadedmetadata', () => {
                video.currentTime = Math.min(0.1, video.duration || 0.1);
            }, { once: true });

            video.addEventListener('seeked', () => {
                buildPoster();
            }, { once: true });

            video.addEventListener('error', () => {
                video.setAttribute('poster', window.__videoFallbackPoster);
            }, { once: true });

            // Icône play overlay (style gallery.html)
            let playIcon = document.createElement('div');
            playIcon.innerHTML = '▶';
            playIcon.style.position = 'absolute';
            playIcon.style.top = '50%';
            playIcon.style.left = '50%';
            playIcon.style.transform = 'translate(-50%, -50%)';
            playIcon.style.fontSize = '3rem';
            playIcon.style.color = 'white';
            playIcon.style.textShadow = '0 0 10px rgba(0,0,0,0.7)';
            playIcon.style.pointerEvents = 'none';
            playIcon.style.opacity = '0.8';

            div.appendChild(video);
            div.appendChild(playIcon);
        } else {
            // Image
            let img = document.createElement('img');
            img.src = 'gallery/' + file;
            img.onclick = () => {
                logEvent('Ouverture galerie (image)', 'info');
                window.location.href = 'galerie/gallery.html';
            }; // Rediriger vers gallery
            div.appendChild(img);
        }

        // Bouton Suppression (Design amélioré)
        let btnDelete = document.createElement('button');
        btnDelete.innerHTML = "&times;"; // <--- MODIFICATION ICI ("×" au lieu de "X")
        btnDelete.className = "btn-delete";
        btnDelete.title = "Supprimer " + (isVideo ? "la vidéo" : "l'image"); // Infobulle au survol
        
        btnDelete.onclick = (e) => {
            e.stopPropagation();
            if(confirm("Supprimer " + file + " ?")) {
                let msg = new ROSLIB.Message({ data: file });
                deletePub.publish(msg);
                logEvent(`Suppression demandée: ${file}`, 'warn');
            }
        };

        div.appendChild(btnDelete);
        grid.appendChild(div);
    });
}

// Ancienne logique de croix rouge désactivée
function handleMapClick(event) {
    console.log("Clic sur carte (fonctionnalité croix désactivée)");
    logEvent('Clic sur la carte', 'info');
}

function toggleFullscreen() {
    const elem = document.getElementById('cameraFeed');
    if (!document.fullscreenElement) elem.requestFullscreen().catch(err => {});
    else document.exitFullscreen();
    logEvent('Plein écran vidéo basculé', 'info');
}

// =======================================================================
// TRAJETS : liste, chargement, dessin (mise à l'échelle simple)
// =======================================================================

function updateTrajectoryList(files) {
    const select = document.getElementById('trajSelect');
    const currentVal = select.value;
    select.innerHTML = '<option value="">-- Choisir un trajet --</option>';
    files.forEach(file => {
        const opt = document.createElement('option');
        opt.value = file;
        opt.text = file;
        select.appendChild(opt);
    });
    if (currentVal) select.value = currentVal;
}

function loadSelectedTrajectory() {
    const filename = document.getElementById('trajSelect').value;
    const info = document.getElementById('trajInfo');
    
    if (!filename) {
        if (info) info.innerText = "Aucun trajet sélectionné";
        logEvent('Trajet non sélectionné', 'warn');
        return;
    }

    if (info) info.innerText = "⏳ Chargement...";

    const url = `trajectories/${filename}`;
    fetch(url)
        .then(response => {
            if (!response.ok) throw new Error("Fichier non trouvé");
            return response.json();
        })
        .then(data => {
            drawTrajectoryOnMap(data);
            if (info) {
                const nbPoints = data.trajectory ? data.trajectory.length : 0;
                info.innerText = `✅ Trajet chargé : ${nbPoints} points`;
            }
            logEvent(`Trajet chargé: ${filename}`, 'success');
        })
        .catch(err => {
            console.error("Erreur chargement json:", err);
            if (info) info.innerText = "❌ Erreur lors du chargement";
            logEvent(`Erreur chargement trajet: ${filename}`, 'error');
        });
}

function drawTrajectoryOnMap(data) {
    const mapArea = document.getElementById('mapArea');
    const polyline = document.getElementById('displayPath');
    const startCircle = document.getElementById('startCircle');
    if (!mapArea || !polyline || !startCircle) return;

    const displayW = mapArea.clientWidth;
    const displayH = mapArea.clientHeight;

    const originalW = data.meta && data.meta.mapDimensions ? data.meta.mapDimensions.width : 1920;
    const originalH = data.meta && data.meta.mapDimensions ? data.meta.mapDimensions.height : 1080;

    const ratioX = displayW / originalW;
    const ratioY = displayH / originalH;

    const startX = data.startPoint && data.startPoint.pixel ? data.startPoint.pixel.x * ratioX : 0;
    const startY = data.startPoint && data.startPoint.pixel ? data.startPoint.pixel.y * ratioY : 0;

    startCircle.setAttribute('cx', startX);
    startCircle.setAttribute('cy', startY);

    let pointsStr = `${startX},${startY}`;
    if (data.trajectory && Array.isArray(data.trajectory)) {
        data.trajectory.forEach(p => {
            const px = p.pixel.x * ratioX;
            const py = p.pixel.y * ratioY;
            pointsStr += ` ${px},${py}`;
        });
    }
    polyline.setAttribute('points', pointsStr);
}

// =======================================================================
// GESTION DE LA MODALE REGLAGES
// =======================================================================

function toggleSettings() {
    const modal = document.getElementById('settingsModal');
    // Si c'est affiché (block), on cache (none), sinon on affiche
    modal.style.display = (modal.style.display === 'block') ? 'none' : 'block';
    logEvent(`Réglages: ${modal.style.display === 'block' ? 'ouverts' : 'fermés'}`, 'info');
}

// Gestion du mode sombre
function toggleDarkMode() {
    document.body.classList.toggle('dark-mode');
    
    // Sauvegarder la préférence
    const isDarkMode = document.body.classList.contains('dark-mode');
    localStorage.setItem('darkMode', isDarkMode ? 'enabled' : 'disabled');
    
    // Changer l'icône
    const btn = document.getElementById('btnDarkMode');
    if (btn) {
        btn.textContent = isDarkMode ? '☀️' : '🌙';
    }
    logEvent(`Mode sombre: ${isDarkMode ? 'activé' : 'désactivé'}`, 'info');
}

// Charger la préférence au démarrage
window.addEventListener('DOMContentLoaded', () => {
    const darkMode = localStorage.getItem('darkMode');
    if (darkMode === 'enabled') {
        document.body.classList.add('dark-mode');
        const btn = document.getElementById('btnDarkMode');
        if (btn) btn.textContent = '☀️';
    }
    logEvent('Interface chargée', 'success');

    // Charger les valeurs sauvegardées des curseurs
    const savedSpeed = localStorage.getItem('robotSpeed');
    if (savedSpeed !== null) {
        const speedSlider = document.getElementById('speedSlider');
        if (speedSlider) {
            speedSlider.value = savedSpeed;
            updateSpeed(savedSpeed);
        }
    }

    const savedZoom = localStorage.getItem('zoomValue');
    if (savedZoom !== null) {
        const zoomSlider = document.getElementById('zoomSlider');
        const zoomSliderModal = document.getElementById('zoomSliderModal');
        if (zoomSlider) zoomSlider.value = savedZoom;
        if (zoomSliderModal) zoomSliderModal.value = savedZoom;
        const zoomVal = document.getElementById('zoomVal');
        const zoomValModal = document.getElementById('zoomValModal');
        if (zoomVal) zoomVal.innerText = savedZoom + '%';
        if (zoomValModal) zoomValModal.innerText = savedZoom + '%';
    }

    const savedArm = localStorage.getItem('armHeight');
    if (savedArm !== null) {
        const armSlider = document.getElementById('armSlider');
        const armSliderModal = document.getElementById('armSliderModal');
        if (armSlider) armSlider.value = savedArm;
        if (armSliderModal) armSliderModal.value = savedArm;
        const armVal = document.getElementById('armVal');
        const armValModal = document.getElementById('armValModal');
        if (armVal) armVal.innerText = savedArm + '%';
        if (armValModal) armValModal.innerText = savedArm + '%';
    }

    const navButton = document.querySelector('.nav-link-button');
    if (navButton) {
        navButton.addEventListener('click', () => {
            logEvent('Ouverture page navigation', 'info');
        });
    }

    const mapArea = document.getElementById('mapArea');
    if (mapArea) {
        mapArea.addEventListener('click', handleMapClick);
    }
});

// Fermer la modale si on clique en dehors de la boîte (sur le fond gris)
window.onclick = function(event) {
    const modal = document.getElementById('settingsModal');
    if (event.target == modal) {
        modal.style.display = "none";
    }
}
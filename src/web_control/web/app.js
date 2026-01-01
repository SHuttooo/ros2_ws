// =======================================================================
// 1. CONFIGURATION & CONNEXION
// =======================================================================
const robotIp = window.location.hostname;
const videoHost = robotIp === "" ? "localhost" : robotIp;

// Vidéo
const videoElement = document.getElementById('cameraFeed');
videoElement.src = `http://${videoHost}:8080/stream?topic=/image_raw&type=mjpeg&quality=100`;

// WebSocket
const ros = new ROSLIB.Ros({ url: `ws://${videoHost}:9090` });

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

// Robot (Point: x, y)
const cmdVelPub = new ROSLIB.Topic({
    ros: ros,
    name: '/robot/cmd_vel_buttons',
    messageType: 'geometry_msgs/Point' // Changé de Twist à Point
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
        if (msg.data !== lastGalleryData) {
            lastGalleryData = msg.data;
            updateGallery(JSON.parse(msg.data)); 
        }
    } catch (e) {} 
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
}

// --- ROBOT (ZQSD) ---
function sendCmd(direction) {
    // 🛑 Arrêt automatique de la mission si on bouge le robot manuellement
    if (missionActive && direction !== 'stop') {
        toggleMission(); // Cela va passer le bouton à "Lancer" et arrêter la mission
        console.log("Mission interrompue par mouvement manuel.");
    }

    // Utilisation de Point (x, y) au lieu de Twist
    let point = new ROSLIB.Message({
        x: 0.0,
        y: 0.0,
        z: 0.0
    });

    if (direction === 'up')    point.x = 1.0 * robotSpeed;
    if (direction === 'down')  point.x = -1.0 * robotSpeed;
    if (direction === 'right') point.y = 1.0 * robotSpeed; // y pour droite
    if (direction === 'left')  point.y = -1.0 * robotSpeed; // y pour gauche

    cmdVelPub.publish(point);
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
}

// --- CLAVIER ---
const keyState = {};

document.addEventListener('keydown', (event) => {
    const key = event.key.toLowerCase();
    if (keyState[key]) return;
    keyState[key] = true;

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
}

function updateArm(val) {
    const elem = document.getElementById('armVal');
    const elemModal = document.getElementById('armValModal');
    if (elem) elem.innerText = val + '%';
    if (elemModal) elemModal.innerText = val + '%';
    localStorage.setItem('armHeight', val);
    armPub.publish(new ROSLIB.Message({ data: parseFloat(val) }));
}

// Gestion Lampe et Micro
let lampActive = false;
let micActive = false;

function toggleLamp() {
    const btn = document.getElementById('btnLamp');
    lampActive = !lampActive;
    if (lampActive) {
        btn.classList.add('active');
    } else {
        btn.classList.remove('active');
    }
}

function toggleMic() {
    const btn = document.getElementById('btnMic');
    micActive = !micActive;
    if (micActive) {
        btn.classList.add('active');
    } else {
        btn.classList.remove('active');
    }
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
    } else {
        btn.innerText = "🚀 Lancer la mission";
        btn.className = "mission-btn start";
        missionPub.publish(new ROSLIB.Message({ data: false }));
    }
}

// Photo & Vidéo
let isRecording = false;
function takePhoto() {
    photoClient.callService(new ROSLIB.ServiceRequest(), (res) => alert(res.success ? "📸 Prise !" : "Erreur"));
}
function toggleVideo() {
    let btn = document.getElementById('btnRecord');
    if (!isRecording) {
        startVideoClient.callService(new ROSLIB.ServiceRequest(), (res) => {
            if(res.success) { isRecording = true; btn.innerText = "⏹ STOP"; btn.style.backgroundColor = "black"; }
        });
    } else {
        stopVideoClient.callService(new ROSLIB.ServiceRequest(), (res) => {
            if(res.success) { isRecording = false; btn.innerText = "🔴 REC"; btn.style.backgroundColor = "#e74c3c"; }
        });
    }
}

// Galerie & Suppression
function updateGallery(files) {
    const grid = document.getElementById('galleryGrid');
    grid.innerHTML = "";
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
            video.onclick = () => window.location.href = 'galerie/gallery.html'; // Rediriger vers gallery

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
            img.onclick = () => window.location.href = 'galerie/gallery.html'; // Rediriger vers gallery
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
            }
        };

        div.appendChild(btnDelete);
        grid.appendChild(div);
    });
}

// Ancienne logique de croix rouge désactivée
function handleMapClick(event) {
    console.log("Clic sur carte (fonctionnalité croix désactivée)");
}

function toggleFullscreen() {
    const elem = document.getElementById('cameraFeed');
    if (!document.fullscreenElement) elem.requestFullscreen().catch(err => {});
    else document.exitFullscreen();
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
        })
        .catch(err => {
            console.error("Erreur chargement json:", err);
            if (info) info.innerText = "❌ Erreur lors du chargement";
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
}

// Charger la préférence au démarrage
window.addEventListener('DOMContentLoaded', () => {
    const darkMode = localStorage.getItem('darkMode');
    if (darkMode === 'enabled') {
        document.body.classList.add('dark-mode');
        const btn = document.getElementById('btnDarkMode');
        if (btn) btn.textContent = '☀️';
    }

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
});

// Fermer la modale si on clique en dehors de la boîte (sur le fond gris)
window.onclick = function(event) {
    const modal = document.getElementById('settingsModal');
    if (event.target == modal) {
        modal.style.display = "none";
    }
}
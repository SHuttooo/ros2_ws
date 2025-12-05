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

const gallerySub = new ROSLIB.Topic({ ros: ros, name: '/ui/gallery_files', messageType: 'std_msgs/String' });
gallerySub.subscribe((msg) => { try { updateGallery(JSON.parse(msg.data)); } catch (e) {} });

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
    zoomPub.publish(new ROSLIB.Message({ data: parseFloat(val) }));
}

function updateArm(val) {
    armPub.publish(new ROSLIB.Message({ data: parseFloat(val) }));
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

        // Image
        let img = document.createElement('img');
        img.src = 'gallery/' + file;
        img.onclick = () => window.open(img.src); // Ouvrir en grand au clic

        // Bouton Suppression (Design amélioré)
        let btnDelete = document.createElement('button');
        btnDelete.innerHTML = "&times;"; // <--- MODIFICATION ICI ("×" au lieu de "X")
        btnDelete.className = "btn-delete";
        btnDelete.title = "Supprimer l'image"; // Infobulle au survol
        
        btnDelete.onclick = (e) => {
            e.stopPropagation();
            if(confirm("Supprimer " + file + " ?")) {
                let msg = new ROSLIB.Message({ data: file });
                deletePub.publish(msg);
            }
        };

        div.appendChild(img);
        div.appendChild(btnDelete);
        grid.appendChild(div);
    });
}

function handleMapClick(event) {
    const rect = document.getElementById('mapArea').getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;
    
    const marker = document.getElementById('clickMarker');
    marker.style.display = 'block';
    marker.style.left = (x - 10) + 'px';
    marker.style.top = (y - 10) + 'px';
    
    document.getElementById('clickCoords').innerText = `x: ${Math.round(x)}, y: ${Math.round(y)}`;
    clickPub.publish(new ROSLIB.Message({ x: x, y: y, z: 0.0 }));
}

function toggleFullscreen() {
    const elem = document.getElementById('cameraFeed');
    if (!document.fullscreenElement) elem.requestFullscreen().catch(err => {});
    else document.exitFullscreen();
}

// =======================================================================
// GESTION DE LA MODALE REGLAGES
// =======================================================================

function toggleSettings() {
    const modal = document.getElementById('settingsModal');
    // Si c'est affiché (block), on cache (none), sinon on affiche
    modal.style.display = (modal.style.display === 'block') ? 'none' : 'block';
}

// Fermer la modale si on clique en dehors de la boîte (sur le fond gris)
window.onclick = function(event) {
    const modal = document.getElementById('settingsModal');
    if (event.target == modal) {
        modal.style.display = "none";
    }
}
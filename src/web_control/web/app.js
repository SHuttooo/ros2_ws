// =======================================================================
// 1. CONFIGURATION & CONNEXION
// =======================================================================
const serverIp = window.location.hostname;  // IP du serveur web
const robotIp = "100.106.79.105";          // IP du robot/Raspberry
const videoHost = serverIp === "" ? "localhost" : serverIp;
const externalStreamUrl = "http://100.106.79.105:8889/mystream/";

// Vidéo WebRTC (faible latence)
const videoElement = document.getElementById('cameraFeed');
const webrtcStatus = document.getElementById('webrtcStatus');

if (videoElement) {
    if (externalStreamUrl) {
        initExternalStream();
    } else {
        initWebRTC();
    }
}

function initExternalStream() {
    const container = videoElement.parentElement;
    if (!container) {
        videoElement.src = externalStreamUrl;
        videoElement.play().catch(() => {});
        if (webrtcStatus) {
            webrtcStatus.textContent = 'Flux externe';
        }
        return;
    }

    const iframe = document.createElement('iframe');
    iframe.src = externalStreamUrl;
    iframe.style.width = '100%';
    iframe.style.height = '100%';
    iframe.style.border = '0';
    iframe.allow = 'autoplay; fullscreen';
    container.replaceChild(iframe, videoElement);
    if (webrtcStatus) {
        if (location.protocol === 'https:' && externalStreamUrl.startsWith('http://')) {
            webrtcStatus.textContent = 'Flux externe (HTTP bloqué en HTTPS)';
        } else {
            webrtcStatus.textContent = 'Flux externe';
        }
    }
}

function initWebRTC() {
    const host = window.location.hostname || "localhost";
    const wsUrl = `ws://${host}:8091/ws`;
    const pc = new RTCPeerConnection({ iceServers: [] });
    const ws = new WebSocket(wsUrl);

    let audioEnabled = false;
    const audioSlider = document.getElementById('audioSlider');

    const setStatus = (text) => {
        if (webrtcStatus) {
            webrtcStatus.textContent = text;
        }
    };

    window.toggleAudio = () => {
        audioEnabled = !audioEnabled;
        videoElement.muted = !audioEnabled;
        console.log('Audio toggled:', audioEnabled, 'Volume:', videoElement.volume, 'Muted:', videoElement.muted);
        if (audioEnabled) {
            videoElement.play().catch(() => {});
        }
        const btn = document.getElementById('btnAudio');
        if (btn) {
            btn.textContent = audioEnabled ? '🔊' : '🔇';
        }
        setStatus(audioEnabled ? 'WebRTC: son activé' : 'WebRTC: son coupé');
        if (audioSlider) {
            audioSlider.value = audioEnabled ? Math.round(videoElement.volume * 100) : 0;
        }
    };

    if (audioSlider) {
        audioSlider.addEventListener('input', () => {
            const volume = Math.max(0, Math.min(1, Number(audioSlider.value) / 100));
            videoElement.volume = volume;
            const btn = document.getElementById('btnAudio');
            if (volume > 0 && videoElement.muted) {
                audioEnabled = true;
                videoElement.muted = false;
                if (btn) {
                    btn.textContent = '🔊';
                }
            } else if (volume === 0) {
                audioEnabled = false;
                videoElement.muted = true;
                if (btn) {
                    btn.textContent = '🔇';
                }
            }
        });
    }

    pc.ontrack = (event) => {
        console.log('WebRTC track reçue:', event.track.kind, event.streams[0]);
        if (videoElement.srcObject !== event.streams[0]) {
            videoElement.srcObject = event.streams[0];
            const stream = event.streams[0];
            console.log('Audio tracks:', stream.getAudioTracks().length);
            console.log('Video tracks:', stream.getVideoTracks().length);
            videoElement.play().catch(() => {});
            setStatus('WebRTC: flux reçu');
        }
    };

    pc.onconnectionstatechange = () => {
        setStatus(`WebRTC: ${pc.connectionState}`);
    };

    pc.oniceconnectionstatechange = () => {
        setStatus(`ICE: ${pc.iceConnectionState}`);
    };

    ws.onopen = async () => {
        setStatus('WebRTC: signalisation...');
        const offer = await pc.createOffer({
            offerToReceiveVideo: true,
            offerToReceiveAudio: true
        });
        await pc.setLocalDescription(offer);
        ws.send(JSON.stringify({ type: 'offer', sdp: offer.sdp }));
    };

    ws.onmessage = async (event) => {
        const data = JSON.parse(event.data);
        if (data.type === 'answer') {
            await pc.setRemoteDescription({ type: 'answer', sdp: data.sdp });
            setStatus('WebRTC: connecté');
        } else if (data.type === 'error') {
            setStatus(`WebRTC: ${data.message}`);
        }
    };

    ws.onerror = () => setStatus('WebRTC: erreur WebSocket');
    ws.onclose = () => setStatus('WebRTC: WebSocket fermé');
}

// WebSocket 1: Se connecte au serveur local (Zehno) pour galerie/trajets/logs
const ros = new ROSLIB.Ros({ url: `ws://${window.location.hostname}:9090` });

// WebSocket 2: Se connecte directement à la Raspberry pour /cmd_vel (commandes robot)
const rosRobot = new ROSLIB.Ros({ url: `ws://${robotIp}:9090` });

let rosServerConnected = false;
let rosRobotConnected = false;

ros.on('connection', () => {
    rosServerConnected = true;
    document.getElementById('status').innerText = 'Connecté';
    document.getElementById('status').className = 'connected';
});
ros.on('error', (error) => {
    rosServerConnected = false;
    document.getElementById('status').innerText = 'Erreur';
    document.getElementById('status').className = 'disconnected';
});
ros.on('close', () => {
    rosServerConnected = false;
    document.getElementById('status').innerText = 'Déconnecté';
    document.getElementById('status').className = 'disconnected';
});

rosRobot.on('connection', () => {
    rosRobotConnected = true;
    console.info('ROS Robot: connecté');
});

rosRobot.on('error', (error) => {
    rosRobotConnected = false;
    console.warn('ROS Robot: erreur connexion', error);
});

rosRobot.on('close', () => {
    rosRobotConnected = false;
    console.warn('ROS Robot: déconnecté');
});

function createRobotPublisher(name, messageType, options = {}) {
    const { robotOnly = false } = options;
    const robotTopic = new ROSLIB.Topic({ ros: rosRobot, name, messageType });
    const serverTopic = robotOnly ? null : new ROSLIB.Topic({ ros, name, messageType });

    return {
        publish(message) {
            let robotSent = false;
            let serverSent = false;

            if (rosRobotConnected || robotOnly) {
                try {
                    robotTopic.publish(message);
                    robotSent = true;
                } catch (err) {
                    console.error(`Erreur publish robot ${name}:`, err);
                }
            }

            if (!robotOnly && rosServerConnected && serverTopic) {
                try {
                    serverTopic.publish(message);
                    serverSent = true;
                } catch (err) {
                    console.error(`Erreur publish serveur ${name}:`, err);
                }
            }

            if (!robotSent && !serverSent) {
                console.warn(`Topic ${name} non envoyé (aucune connexion disponible)`);
            }
        }
    };
}

// =======================================================================
// 2. TOPICS
// =======================================================================

// Robot (TwistStamped - type imposé par un autre node)
const cmdVelPub = createRobotPublisher('/cmd_vel', 'geometry_msgs/TwistStamped');

// PTZ (Point: x, y)
const ptzPub = createRobotPublisher('/camera/ptz', 'geometry_msgs/Point');

// Mission Status
const missionPub = createRobotPublisher('/mission/status', 'std_msgs/Bool');

// Delete Image (Nouveau)
const deletePub = createRobotPublisher('/camera/delete_image', 'std_msgs/String');

const zoomPub = createRobotPublisher('/camera/zoom', 'std_msgs/Float32');
const focusPub = createRobotPublisher('/camera/focus', 'std_msgs/Float32');
const autofocusPub = createRobotPublisher('/camera/autofocus', 'std_msgs/Bool');
const lightPub = createRobotPublisher('/camera/light', 'std_msgs/Bool');
const alertPub = createRobotPublisher('/camera/alert', 'std_msgs/Bool');
const robotVolumePub = createRobotPublisher('/robot/volume', 'std_msgs/Float32');
const armSpeedPub = createRobotPublisher('/robot/arm_speed', 'std_msgs/Float32');
const armPosPub = createRobotPublisher('/robot/arm_position', 'std_msgs/Float32');
const clickPub = createRobotPublisher('/ui/click', 'geometry_msgs/Point');

// Logs UI
const logPub = createRobotPublisher('/ui/system_logs', 'std_msgs/String');

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
const emergencyPub = createRobotPublisher('/robot/emergency_stop', 'std_msgs/Bool');

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

function updateFocus(val) {
    const elemModal = document.getElementById('focusValModal');
    const focusPos = Math.max(0, Math.min(28, parseInt(val)));

    setAutofocusState(false, { publish: true });
    if (elemModal) elemModal.innerText = focusPos;
    logEvent(`Focus manuel: ${focusPos}`, 'info');
    localStorage.setItem('focusValue', String(focusPos));
    focusPub.publish(new ROSLIB.Message({ data: focusPos }));
}

let autofocusEnabled = true;

function setAutofocusState(enabled, options = {}) {
    autofocusEnabled = enabled;
    localStorage.setItem('autofocusEnabled', enabled ? '1' : '0');

    const btn = document.getElementById('btnAutofocus');
    if (btn) {
        btn.textContent = enabled ? '🎯 Autofocus: ON' : '🎯 Autofocus: OFF';
        btn.classList.toggle('active', enabled);
    }

    const focusSliderModal = document.getElementById('focusSliderModal');
    if (focusSliderModal) {
        focusSliderModal.disabled = enabled;
    }

    if (enabled && options.publish) {
        const focusValModal = document.getElementById('focusValModal');
        if (focusValModal) focusValModal.innerText = 'Auto';
        localStorage.setItem('focusValue', '0');
        autofocusPub.publish(new ROSLIB.Message({ data: true }));
    } else if (!enabled && options.publish) {
        autofocusPub.publish(new ROSLIB.Message({ data: false }));
    }
}

function toggleAutofocus() {
    setAutofocusState(!autofocusEnabled, { publish: true });
}

function updateArmSpeed(val) {
    const elem = document.getElementById('armSpeedVal');
    if (elem) elem.innerText = val + '%';
    localStorage.setItem('armSpeed', val);
    armSpeedPub.publish(new ROSLIB.Message({ data: parseFloat(val) }));
    logEvent(`Vitesse bras: ${val}%`, 'info');
}

function updateArmPos(val) {
    const elem = document.getElementById('armPosVal');
    if (elem) elem.innerText = val + '%';
    localStorage.setItem('armPosition', val);
    armPosPub.publish(new ROSLIB.Message({ data: parseFloat(val) }));
    logEvent(`Position bras: ${val}%`, 'info');
}

function updateRobotVolume(val) {
    const elem = document.getElementById('robotVolumeVal');
    if (elem) elem.innerText = val + '%';
    localStorage.setItem('robotVolume', val);
    robotVolumePub.publish(new ROSLIB.Message({ data: parseFloat(val) }));
    logEvent(`Volume robot: ${val}%`, 'info');
}

function triggerAlert() {
    const btn = document.getElementById('btnAlert');
    alertPub.publish(new ROSLIB.Message({ data: true }));
    if (btn) {
        btn.classList.add('active');
        btn.textContent = '🚨 Alerte Caméra: TEST';
    }
    logEvent('Alerte caméra: TEST', 'warning');

    setTimeout(() => {
        alertPub.publish(new ROSLIB.Message({ data: false }));
        if (btn) {
            btn.classList.remove('active');
            btn.textContent = '🚨 Alerte Caméra';
        }
    }, 1000);
}

// Gestion Lampe et Micro
let lampActive = false;
let micActive = false;

function toggleLamp() {
    const btn = document.getElementById('btnLamp');
    lampActive = !lampActive;
    
    console.log('toggleLamp() appelé, lampActive =', lampActive);
    
    // Publier sur /camera/light
    const lightMsg = new ROSLIB.Message({ data: lampActive });
    lightPub.publish(lightMsg);
    console.log('Message publié sur /camera/light:', lightMsg);
    
    if (lampActive) {
        btn.classList.add('active');
        logEvent('💡 Lampe IR: activée', 'success');
    } else {
        btn.classList.remove('active');
        logEvent('💡 Lampe IR: désactivée', 'warn');
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
let mediaRecorder = null;
let recordedChunks = [];

async function uploadBlob(blob, endpoint, filename) {
    const params = new URLSearchParams({ filename });
    const res = await fetch(`${endpoint}?${params.toString()}`, {
        method: 'POST',
        body: blob
    });
    if (!res.ok) {
        throw new Error(`Upload failed: ${res.status}`);
    }
}

function takePhoto() {
    if (videoElement && videoElement.srcObject && videoElement.videoWidth) {
        const canvas = document.createElement('canvas');
        canvas.width = videoElement.videoWidth;
        canvas.height = videoElement.videoHeight;
        const ctx = canvas.getContext('2d');
        ctx.drawImage(videoElement, 0, 0, canvas.width, canvas.height);
        canvas.toBlob(async (blob) => {
            if (blob) {
                const filename = `photo_${Date.now()}.jpg`;
                try {
                    await uploadBlob(blob, '/upload_photo', filename);
                    logEvent('Photo prise (WebRTC)', 'success');
                } catch (e) {
                    logEvent('Erreur upload photo (WebRTC)', 'error');
                }
            } else {
                logEvent('Erreur prise photo (WebRTC)', 'error');
            }
        }, 'image/jpeg', 0.95);
        return;
    }

    logEvent('Photo demandée (ROS2)', 'info');
    photoClient.callService(new ROSLIB.ServiceRequest(), (res) => {
        alert(res.success ? "📸 Prise !" : "Erreur");
        logEvent(res.success ? 'Photo prise' : 'Erreur prise photo', res.success ? 'success' : 'error');
    });
}

function toggleVideo() {
    let btn = document.getElementById('btnRecord');

    if (videoElement && videoElement.srcObject && window.MediaRecorder) {
        if (!isRecording) {
            recordedChunks = [];
            mediaRecorder = new MediaRecorder(videoElement.srcObject, { mimeType: 'video/webm;codecs=vp8' });
            mediaRecorder.ondataavailable = (event) => {
                if (event.data && event.data.size > 0) {
                    recordedChunks.push(event.data);
                }
            };
            mediaRecorder.onstop = async () => {
                const blob = new Blob(recordedChunks, { type: 'video/webm' });
                recordedChunks = [];
                try {
                    await uploadBlob(blob, '/upload_video', `video_${Date.now()}.webm`);
                    logEvent('Vidéo sauvegardée (WebRTC)', 'success');
                } catch (e) {
                    logEvent('Erreur upload vidéo (WebRTC)', 'error');
                }
            };
            mediaRecorder.start();
            isRecording = true;
            btn.innerText = "⏹ STOP";
            btn.style.backgroundColor = "black";
            logEvent('Enregistrement vidéo démarré (WebRTC)', 'success');
        } else {
            mediaRecorder.stop();
            isRecording = false;
            btn.innerText = "🔴 REC";
            btn.style.backgroundColor = "#e74c3c";
            logEvent('Enregistrement vidéo arrêté (WebRTC)', 'success');
        }
        return;
    }

    if (!isRecording) {
        logEvent('Démarrage enregistrement vidéo (ROS2)', 'info');
        startVideoClient.callService(new ROSLIB.ServiceRequest(), (res) => {
            if (res.success) {
                isRecording = true;
                btn.innerText = "⏹ STOP";
                btn.style.backgroundColor = "black";
            }
            logEvent(res.success ? 'Enregistrement vidéo démarré' : 'Erreur démarrage vidéo', res.success ? 'success' : 'error');
        });
    } else {
        logEvent('Arrêt enregistrement vidéo (ROS2)', 'info');
        stopVideoClient.callService(new ROSLIB.ServiceRequest(), (res) => {
            if (res.success) {
                isRecording = false;
                btn.innerText = "🔴 REC";
                btn.style.backgroundColor = "#e74c3c";
            }
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
        const isVideo = file.toLowerCase().endsWith('.mp4') || file.toLowerCase().endsWith('.avi') || file.toLowerCase().endsWith('.mov') || file.toLowerCase().endsWith('.webm');

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
    const container = document.getElementById('videoContainer');
    const elem = container || document.getElementById('cameraFeed');
    if (!elem) return;
    if (!document.fullscreenElement) {
        elem.requestFullscreen().catch(err => {});
    } else {
        document.exitFullscreen();
    }
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

    const savedFocus = localStorage.getItem('focusValue') || '0';
    const focusSliderModal = document.getElementById('focusSliderModal');
    const normalizedFocus = Math.max(0, Math.min(28, parseInt(savedFocus)) || 0);
    if (focusSliderModal) focusSliderModal.value = normalizedFocus;
    const focusValModal = document.getElementById('focusValModal');
    if (focusValModal) {
        if (normalizedFocus === 0) {
            focusValModal.innerText = 'Auto';
        } else {
            focusValModal.innerText = normalizedFocus;
        }
    }

    const savedAutofocus = localStorage.getItem('autofocusEnabled');
    const autofocusFromStorage = savedAutofocus === null ? (normalizedFocus === 0) : savedAutofocus === '1';
    setAutofocusState(autofocusFromStorage, { publish: false });
    if (!autofocusFromStorage && normalizedFocus !== 0) {
        const focusValModal = document.getElementById('focusValModal');
        if (focusValModal) focusValModal.innerText = normalizedFocus;
    }

    const savedArmSpeed = localStorage.getItem('armSpeed') || '50';
    const armSpeedSlider = document.getElementById('armSpeedSlider');
    if (armSpeedSlider) armSpeedSlider.value = savedArmSpeed;
    const armSpeedVal = document.getElementById('armSpeedVal');
    if (armSpeedVal) armSpeedVal.innerText = savedArmSpeed + '%';

    const savedArmPos = localStorage.getItem('armPosition') || '0';
    const armPosSlider = document.getElementById('armPosSlider');
    if (armPosSlider) armPosSlider.value = savedArmPos;
    const armPosVal = document.getElementById('armPosVal');
    if (armPosVal) armPosVal.innerText = savedArmPos + '%';

    const savedRobotVolume = localStorage.getItem('robotVolume') || '50';
    const robotVolumeSlider = document.getElementById('robotVolumeSlider');
    if (robotVolumeSlider) robotVolumeSlider.value = savedRobotVolume;
    const robotVolumeVal = document.getElementById('robotVolumeVal');
    if (robotVolumeVal) robotVolumeVal.innerText = savedRobotVolume + '%';

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
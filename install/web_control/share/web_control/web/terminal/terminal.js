const robotIp = window.location.hostname;
const videoHost = robotIp === "" ? "localhost" : robotIp;
const ros = new ROSLIB.Ros({ url: `ws://${videoHost}:9090` });

ros.on('connection', () => {
    document.getElementById('status').innerText = 'Connecté';
    document.getElementById('status').className = 'connected';
    addLog('✅ Connexion ROS2 établie', 'success');
});

ros.on('error', () => {
    document.getElementById('status').innerText = 'Erreur';
    document.getElementById('status').className = 'disconnected';
    addLog('❌ Erreur de connexion ROS2', 'error');
});

ros.on('close', () => {
    document.getElementById('status').innerText = 'Déconnecté';
    document.getElementById('status').className = 'disconnected';
    addLog('⚠️ Connexion ROS2 fermée', 'warn');
});

// Topic pour recevoir les logs système
const logsSub = new ROSLIB.Topic({
    ros: ros,
    name: '/ui/system_logs',
    messageType: 'std_msgs/String'
});

let isPaused = false;
let logCount = 0;

logsSub.subscribe((msg) => {
    if (!isPaused) {
        try {
            const logData = JSON.parse(msg.data);
            addLog(logData.message, logData.level || 'info');
        } catch (e) {
            // Si ce n'est pas du JSON, afficher tel quel
            addLog(msg.data, 'info');
        }
    }
});

function addLog(message, level = 'info') {
    const terminal = document.getElementById('terminalWindow');
    const logLine = document.createElement('div');
    logLine.className = `log-line ${level}`;
    
    // Ajouter timestamp
    const now = new Date();
    const timestamp = `[${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}:${now.getSeconds().toString().padStart(2, '0')}]`;
    
    logLine.textContent = `${timestamp} ${message}`;
    terminal.appendChild(logLine);
    
    // Auto-scroll vers le bas
    terminal.scrollTop = terminal.scrollHeight;
    
    // Limiter à 500 lignes max
    const lines = terminal.getElementsByClassName('log-line');
    if (lines.length > 500) {
        terminal.removeChild(lines[0]);
    }
    
    // Mettre à jour le compteur
    logCount = lines.length;
    document.getElementById('logCount').textContent = logCount;
}

function clearLogs() {
    const terminal = document.getElementById('terminalWindow');
    terminal.innerHTML = '';
    logCount = 0;
    document.getElementById('logCount').textContent = logCount;
    addLog('Terminal effacé', 'info');
}

function togglePause() {
    isPaused = !isPaused;
    const btn = document.getElementById('btnPause');
    if (isPaused) {
        btn.textContent = '▶️ Reprendre';
        btn.classList.add('paused');
        addLog('⏸️ Pause activée', 'warn');
    } else {
        btn.textContent = '⏸️ Pause';
        btn.classList.remove('paused');
        addLog('▶️ Logs repris', 'success');
    }
}

function toggleDarkMode() {
    document.body.classList.toggle('dark-mode');
    const isDarkMode = document.body.classList.contains('dark-mode');
    localStorage.setItem('darkMode', isDarkMode ? 'enabled' : 'disabled');
    const btn = document.getElementById('btnDarkMode');
    if (btn) btn.textContent = isDarkMode ? '☀️' : '🌙';
}

window.addEventListener('DOMContentLoaded', () => {
    const darkMode = localStorage.getItem('darkMode');
    if (darkMode === 'enabled') {
        document.body.classList.add('dark-mode');
        const btn = document.getElementById('btnDarkMode');
        if (btn) btn.textContent = '☀️';
    }
});

// Simuler quelques logs au démarrage
setTimeout(() => {
    addLog('Initialisation des modules...', 'info');
}, 1000);

setTimeout(() => {
    addLog('Backend prêt sur le port 8000', 'success');
}, 1500);

setTimeout(() => {
    addLog('GalleryManager démarré', 'success');
}, 2000);

setTimeout(() => {
    addLog('CaptureManager prêt', 'success');
}, 2500);

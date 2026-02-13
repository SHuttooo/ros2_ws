// --- Cookie utility functions ---
function setCookie(name, value, days) {
    const expires = new Date();
    expires.setTime(expires.getTime() + days * 24 * 60 * 60 * 1000);
    document.cookie = `${name}=${value};expires=${expires.toUTCString()};path=/`;
}

function getCookie(name) {
    const nameEQ = name + '=';
    const ca = document.cookie.split(';');
    for (let i = 0; i < ca.length; i++) {
        let c = ca[i];
        while (c.charAt(0) === ' ') c = c.substring(1, c.length);
        if (c.indexOf(nameEQ) === 0) return c.substring(nameEQ.length, c.length);
    }
    return null;
}

document.addEventListener('DOMContentLoaded', () => {
    // --- DOM Elements ---
    const mapContainer = document.getElementById('map-container');
    const mapWorld = document.getElementById('map-world');
    const mapImage = document.getElementById('map-image');
    const trajectoryPath = document.getElementById('trajectory-path');
    const gpsCoordsDisplay = document.getElementById('gps-coords');
    const pointList = document.getElementById('point-list');
    const clearTrajectoryBtn = document.getElementById('clear-trajectory');
    const removeLastPointBtn = document.getElementById('remove-last-point');
    const toggleEditModeBtn = document.getElementById('toggle-edit-mode');
    const clearForbiddenAreasBtn = document.getElementById('clear-forbidden-areas');
    const resetForbiddenAreasBtn = document.getElementById('reset-forbidden-areas');
    const saveTrajectoryBtn = document.getElementById('save-trajectory');
    const trajectoryButtons = document.getElementById('trajectory-buttons');
    const editButtons = document.getElementById('edit-buttons');
    const savedTrajectoriesList = document.getElementById('saved-trajectories-list');
    const zoomInBtn = document.getElementById('zoom-in');
    const zoomOutBtn = document.getElementById('zoom-out');
    const darkModeToggle = document.getElementById('dark-mode-toggle');
    const headerSettingsBtn = document.getElementById('header-settings-btn');
    const headerFontSizeSlider = document.getElementById('header-font-size-slider');
    const headerFontSizeLabel = document.getElementById('header-font-size-label');
    const headerToggleEditModeBtn = document.getElementById('header-toggle-edit-mode');
    const headerClearForbiddenAreasBtn = document.getElementById('header-clear-forbidden-areas');
    const headerResetForbiddenAreasBtn = document.getElementById('header-reset-forbidden-areas');
    const headerEditButtons = document.getElementById('header-edit-buttons');

    // --- ROS Connection ---
    const ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090' // Adjust if your ROS bridge is elsewhere
    });
    ros.on('connection', () => console.log('Connected to websocket server.'));
    ros.on('error', (error) => console.log('Error connecting to websocket server: ', error));
    ros.on('close', () => console.log('Connection to websocket server closed.'));

    // --- ROS Topic ---
    const trajectoryTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/web_trajectory',
        messageType: 'geometry_msgs/PoseArray'
    });
    
    // Topic pour sauvegarder les trajectoires
    const saveTrajectoryPub = new ROSLIB.Topic({
        ros: ros,
        name: '/ui/save_trajectory',
        messageType: 'std_msgs/String'
    });
    
    // Topic pour supprimer une trajectoire
    const deleteTrajectoryPub = new ROSLIB.Topic({
        ros: ros,
        name: '/ui/delete_trajectory',
        messageType: 'std_msgs/String'
    });
    
    // Topic pour recevoir la liste des trajectoires
    const trajListSub = new ROSLIB.Topic({
        ros: ros,
        name: '/ui/trajectory_files',
        messageType: 'std_msgs/String'
    });
    
    // S'abonner à la liste des trajectoires
    trajListSub.subscribe((msg) => {
        try {
            const files = JSON.parse(msg.data);
            updateSavedTrajectoriesList(files);
        } catch (e) {
            console.error('Erreur parsing liste trajectoires:', e);
        }
    });

    // --- State ---
    let trajectoryPoints = [];
    let pointIdCounter = 0;
    let scale = 1;
    let panning = false;
    let isDragging = false;
    let view = { x: 0, y: 0 };
    let start = { x: 0, y: 0 };
    let startClick = { x: 0, y: 0 };
    
    // Zones interdites
    let editMode = false;
    let forbiddenAreas = [];
    let tempRectPoint = null;
    
    // START POINT que l'on a fixé
    const startPoint = { x: 100, y: 100, gps: { lat: 0, lon: 0 }, type: 'start' }; 

    // --- Map Setup ---
    mapImage.src = './map.jpg';
    mapImage.onload = () => {
        // Calculate initial scale to cover the container
        const containerRect = mapContainer.getBoundingClientRect();
        const imgWidth = mapImage.naturalWidth;
        const imgHeight = mapImage.naturalHeight;

        const scaleX = containerRect.width / imgWidth;
        const scaleY = containerRect.height / imgHeight;
        
        // "Cover" behavior: take the larger scale
        scale = Math.max(scaleX, scaleY);

        // Center the image
        view.x = (containerRect.width - imgWidth * scale) / 2;
        view.y = (containerRect.height - imgHeight * scale) / 2;

        // Initialize start point GPS
        startPoint.gps = pixelToGps(startPoint.x, startPoint.y, imgWidth, imgHeight);
        drawPointOnMap(startPoint);

        updateTransform();
        loadForbiddenAreas();
        // La liste des trajectoires sera chargée via ROS topic
    };


    // --- Zoom and Pan Logic ---
    function updateTransform() {
        // Apply transform to the world wrapper, not individual elements
        const transformValue = `translate(${view.x}px, ${view.y}px) scale(${scale})`;
        mapWorld.style.transform = transformValue;
    }

    function resetView() {
        // Reset to cover logic (same as onload)
        const containerRect = mapContainer.getBoundingClientRect();
        const imgWidth = mapImage.naturalWidth;
        const imgHeight = mapImage.naturalHeight;
        const scaleX = containerRect.width / imgWidth;
        const scaleY = containerRect.height / imgHeight;
        scale = Math.max(scaleX, scaleY);
        view.x = (containerRect.width - imgWidth * scale) / 2;
        view.y = (containerRect.height - imgHeight * scale) / 2;
        updateTransform();
    }

    mapContainer.addEventListener('wheel', (event) => {
        event.preventDefault();
        const rect = mapContainer.getBoundingClientRect();
        // Mouse position relative to container
        const mouseX = event.clientX - rect.left;
        const mouseY = event.clientY - rect.top;

        // Mouse position relative to world (before zoom)
        const worldX = (mouseX - view.x) / scale;
        const worldY = (mouseY - view.y) / scale;

        const delta = -event.deltaY;
        const newScale = scale * (1 + delta / 1000);
        
        // Limit zoom (e.g., 0.1x to 10x)
        scale = Math.min(Math.max(0.1, newScale), 10); 

        // Adjust view so mouse stays on same world point
        view.x = mouseX - worldX * scale;
        view.y = mouseY - worldY * scale;

        updateTransform();
    });

    zoomInBtn.addEventListener('click', () => {
        const rect = mapContainer.getBoundingClientRect();
        const centerX = rect.width / 2;
        const centerY = rect.height / 2;
        
        const worldX = (centerX - view.x) / scale;
        const worldY = (centerY - view.y) / scale;

        scale = Math.min(scale * 1.2, 10);
        
        view.x = centerX - worldX * scale;
        view.y = centerY - worldY * scale;
        updateTransform();
    });

    zoomOutBtn.addEventListener('click', () => {
        const rect = mapContainer.getBoundingClientRect();
        const centerX = rect.width / 2;
        const centerY = rect.height / 2;
        
        const worldX = (centerX - view.x) / scale;
        const worldY = (centerY - view.y) / scale;

        scale = Math.max(scale / 1.2, 0.1);
        
        view.x = centerX - worldX * scale;
        view.y = centerY - worldY * scale;
        updateTransform();
    });

    mapContainer.addEventListener('mousedown', (event) => {
        // Ignore mousedown on settings
        if (event.target.closest('.settings-container')) return;
        
        isDragging = false;
        // Only left click for panning
        if (event.button !== 0) return;
        event.preventDefault();
        panning = true;
        mapContainer.style.cursor = 'grabbing';
        start = { x: event.clientX - view.x, y: event.clientY - view.y };
        startClick = { x: event.clientX, y: event.clientY };
    });

    window.addEventListener('mouseup', () => {
        panning = false;
        mapContainer.style.cursor = 'grab';
    });

    window.addEventListener('mousemove', (event) => {
        if (!panning) return;
        event.preventDefault();

        const moveX = event.clientX - startClick.x;
        const moveY = event.clientY - startClick.y;
        if (Math.sqrt(moveX * moveX + moveY * moveY) > 5) {
            isDragging = true;
        }

        view.x = event.clientX - start.x;
        view.y = event.clientY - start.y;
        updateTransform();
    });

    // --- Keyboard Events ---
    window.addEventListener('keydown', (event) => {
        if (event.key.toLowerCase() === 'a' && editMode) {
            event.preventDefault();
            // Trigger a click at the center of the viewport
            const rect = mapContainer.getBoundingClientRect();
            const centerX = rect.width / 2;
            const centerY = rect.height / 2;
            
            // Calculate image coordinates
            const imageX = (centerX - view.x) / scale;
            const imageY = (centerY - view.y) / scale;
            
            if (imageX >= 0 && imageX <= mapImage.naturalWidth && imageY >= 0 && imageY <= mapImage.naturalHeight) {
                handleRectanglePoint(imageX, imageY);
            }
        }
    });


    // --- Point Selection ---
    // Handle Right Click (Context Menu)
    mapContainer.addEventListener('contextmenu', (event) => {
        event.preventDefault(); // Prevent default context menu
        if (isDragging) return;

        const rect = mapContainer.getBoundingClientRect();
        const mouseX = event.clientX - rect.left;
        const mouseY = event.clientY - rect.top;

        const imageX = (mouseX - view.x) / scale;
        const imageY = (mouseY - view.y) / scale;

        if (imageX < 0 || imageX > mapImage.naturalWidth || imageY < 0 || imageY > mapImage.naturalHeight) {
            return;
        }

        // If in edit mode, check if clicking on a forbidden area to delete it
        if (editMode) {
            const clickedAreaIndex = findForbiddenAreaAt(imageX, imageY);
            if (clickedAreaIndex !== -1) {
                // Remove the area
                forbiddenAreas.splice(clickedAreaIndex, 1);
                // Redraw all areas
                redrawForbiddenAreas();
            }
            return;
        }

        const gps = pixelToGps(imageX, imageY, mapImage.naturalWidth, mapImage.naturalHeight);
        
        // Add point with photography attribute
        const point = { 
            id: pointIdCounter++, 
            x: imageX, 
            y: imageY, 
            gps, 
            type: 'photography',
            photography: 'yes' 
        };
        trajectoryPoints.push(point);

        addPointToVisualList(point);
        drawPointOnMap(point);
    });

    mapContainer.addEventListener('click', (event) => {
        // Ignore clicks on buttons and settings
        if (event.target.tagName === 'BUTTON' || event.target.closest('.settings-container')) return;
        
        if (isDragging) return;
        
        // If in edit mode, handle rectangle points instead
        if (editMode) {
            const rect = mapContainer.getBoundingClientRect();
            const mouseX = event.clientX - rect.left;
            const mouseY = event.clientY - rect.top;
            const imageX = (mouseX - view.x) / scale;
            const imageY = (mouseY - view.y) / scale;
            
            if (imageX >= 0 && imageX <= mapImage.naturalWidth && imageY >= 0 && imageY <= mapImage.naturalHeight) {
                handleRectanglePoint(imageX, imageY);
            }
            return;
        }
        
        // Let's calculate coordinates
        const rect = mapContainer.getBoundingClientRect();
        const mouseX = event.clientX - rect.left;
        const mouseY = event.clientY - rect.top;

        // Convert to image coordinates
        const imageX = (mouseX - view.x) / scale;
        const imageY = (mouseY - view.y) / scale;

        // Check bounds (optional, if we want to restrict points to image area)
        if (imageX < 0 || imageX > mapImage.naturalWidth || imageY < 0 || imageY > mapImage.naturalHeight) {
            // Clicked outside the image
            return;
        }

        const gps = pixelToGps(imageX, imageY, mapImage.naturalWidth, mapImage.naturalHeight);
        gpsCoordsDisplay.textContent = `Lat: ${gps.lat.toFixed(6)}, Lon: ${gps.lon.toFixed(6)}`;

        const point = { id: pointIdCounter++, x: imageX, y: imageY, gps, type: 'default' };
        trajectoryPoints.push(point);

        addPointToVisualList(point);
        drawPointOnMap(point);
    });

    // --- Trajectory Management ---
    clearTrajectoryBtn.addEventListener('click', () => {
        trajectoryPoints = [];
        pointList.innerHTML = '';
        gpsCoordsDisplay.textContent = '';
        pointIdCounter = 0;
        
        // Remove only trajectory markers, keep start point
        const markers = mapWorld.querySelectorAll('.map-marker:not(.start)');
        markers.forEach(marker => marker.remove());
        
        // Reset path to just start point
        updateTrajectoryPath();
    });

    removeLastPointBtn.addEventListener('click', () => {
        if (trajectoryPoints.length === 0) {
            alert('Aucun point à supprimer.');
            return;
        }
        
        // Remove the last point
        const lastPoint = trajectoryPoints.pop();
        
        // Remove from list
        const listItems = pointList.querySelectorAll('li');
        if (listItems.length > 0) {
            pointList.removeChild(listItems[listItems.length - 1]);
        }
        
        // Remove marker
        const markers = mapWorld.querySelectorAll('.map-marker:not(.start)');
        if (markers.length > 0) {
            mapWorld.removeChild(markers[markers.length - 1]);
        }
        
        // Update path
        updateTrajectoryPath();
        
        // Update GPS display if no points left
        if (trajectoryPoints.length === 0) {
            gpsCoordsDisplay.textContent = '';
        }
    });

    toggleEditModeBtn.addEventListener('click', () => {
        editMode = !editMode;
        toggleEditModeBtn.textContent = editMode ? 'Mode Édition: ON' : 'Mode Édition: OFF';
        toggleEditModeBtn.classList.toggle('active', editMode);
        toggleEditModeBtn.style.backgroundColor = editMode ? '#28a745' : '#dc3545';
        
        // Toggle visibility of button groups
        trajectoryButtons.style.display = editMode ? 'none' : 'block';
        editButtons.style.display = editMode ? 'block' : 'none';
        
        if (!editMode && tempRectPoint) {
            // Clear temporary point if exiting edit mode
            const tempMarkers = mapWorld.querySelectorAll('.forbidden-area-point');
            tempMarkers.forEach(m => m.remove());
            tempRectPoint = null;
        }
    });

    clearForbiddenAreasBtn.addEventListener('click', () => {
        if (confirm('Voulez-vous vraiment effacer toutes les zones interdites ?')) {
            forbiddenAreas = [];
            const rects = mapWorld.querySelectorAll('.forbidden-area');
            rects.forEach(r => r.remove());
        }
    });

    resetForbiddenAreasBtn.addEventListener('click', () => {
        loadBlankAreas();
    });

    saveTrajectoryBtn.addEventListener('click', () => {
        saveTrajectory();
    });

    // --- UI Update Functions ---
    function addPointToVisualList(point) {
        const li = document.createElement('li');
        let typeStr = "";
        if (point.type === 'photography') {
            typeStr = " [PHOTO]";
            li.style.color = "green";
        }
        li.textContent = `Point ${point.id}${typeStr}: (Lat: ${point.gps.lat.toFixed(4)}, Lon: ${point.gps.lon.toFixed(4)})`;
        pointList.appendChild(li);
    }

    function drawPointOnMap(point) {
        const marker = document.createElement('div');
        marker.className = 'map-marker';
        
        if (point.type === 'photography') {
            marker.classList.add('photography');
        } else if (point.type === 'start') {
            marker.classList.add('start');
        }

        // Position is relative to the map-world wrapper
        marker.style.left = `${point.x}px`;
        marker.style.top = `${point.y}px`;
        mapWorld.appendChild(marker);

        // Update the trajectory path
        updateTrajectoryPath();
    }

    function updateTrajectoryPath() {
        // Start path from startPoint
        let pointsStr = `${startPoint.x},${startPoint.y}`;
        
        if (trajectoryPoints.length > 0) {
            const trajStr = trajectoryPoints.map(p => `${p.x},${p.y}`).join(' ');
            pointsStr += ' ' + trajStr;
        }
        
        trajectoryPath.setAttribute('points', pointsStr);
    }

    // --- Forbidden Areas Management ---
    function handleRectanglePoint(x, y) {
        if (!tempRectPoint) {
            // First point
            tempRectPoint = { x, y };
            // Draw temporary point
            const marker = document.createElement('div');
            marker.className = 'forbidden-area-point';
            marker.style.left = `${x}px`;
            marker.style.top = `${y}px`;
            mapWorld.appendChild(marker);
        } else {
            // Second point - create rectangle
            const x1 = Math.min(tempRectPoint.x, x);
            const y1 = Math.min(tempRectPoint.y, y);
            const x2 = Math.max(tempRectPoint.x, x);
            const y2 = Math.max(tempRectPoint.y, y);
            
            const gps1 = pixelToGps(x1, y1, mapImage.naturalWidth, mapImage.naturalHeight);
            const gps2 = pixelToGps(x2, y2, mapImage.naturalWidth, mapImage.naturalHeight);
            
            const area = {
                x1, y1, x2, y2,
                gps1, gps2
            };
            
            forbiddenAreas.push(area);
            drawForbiddenArea(area);
            
            // Clear temporary point
            const tempMarkers = mapWorld.querySelectorAll('.forbidden-area-point');
            tempMarkers.forEach(m => m.remove());
            tempRectPoint = null;
        }
    }

    function drawForbiddenArea(area) {
        const rect = document.createElement('div');
        rect.className = 'forbidden-area';
        rect.style.left = `${area.x1}px`;
        rect.style.top = `${area.y1}px`;
        rect.style.width = `${area.x2 - area.x1}px`;
        rect.style.height = `${area.y2 - area.y1}px`;
        mapWorld.appendChild(rect);
    }

    function saveTrajectory() {
        // Récupérer le nom de la trajectoire
        const nameInput = document.getElementById('trajectory-name');
        let trajName = nameInput ? nameInput.value.trim() : '';
        
        if (!trajName) {
            alert('⚠️ Veuillez entrer un nom pour la trajectoire');
            return;
        }
        
        // Nettoyer le nom (enlever les caractères spéciaux)
        trajName = trajName.replace(/[^a-zA-Z0-9_-]/g, '_');
        
        // Créer les données de trajectoire
        const imgWidth = mapImage.naturalWidth;
        const imgHeight = mapImage.naturalHeight;
        
        const data = {
            meta: {
                name: trajName,
                timestamp: new Date().toISOString(),
                mapDimensions: {
                    width: imgWidth,
                    height: imgHeight
                }
            },
            startPoint: {
                pixel: { x: startPoint.x, y: startPoint.y },
                gps: startPoint.gps
            },
            trajectory: trajectoryPoints.map(p => ({
                id: p.id,
                pixel: { x: p.x, y: p.y },
                gps: p.gps,
                type: p.type,
                photography: p.photography || 'no'
            })),
            forbiddenAreas: forbiddenAreas.map(area => ({
                pixel: {
                    topLeft: { x: area.x1, y: area.y1 },
                    bottomRight: { x: area.x2, y: area.y2 }
                },
                gps: {
                    topLeft: area.gps1,
                    bottomRight: area.gps2
                }
            }))
        };
        
        // Publier via ROS pour sauvegarde sur le serveur
        const jsonString = JSON.stringify(data);
        const msg = new ROSLIB.Message({
            data: jsonString
        });
        
        saveTrajectoryPub.publish(msg);
        console.log('Trajectoire envoyée pour sauvegarde via ROS');
        alert(`✅ Trajectoire "${trajName}" sauvegardée !`);
        
        // Vider le champ de saisie
        if (nameInput) nameInput.value = '';
    }

    function updateSavedTrajectoriesList(files) {
        savedTrajectoriesList.innerHTML = '';
        
        if (!files || files.length === 0) {
            const li = document.createElement('li');
            li.textContent = 'Aucun trajet sauvegardé';
            li.style.fontStyle = 'italic';
            li.style.color = '#999';
            savedTrajectoriesList.appendChild(li);
            return;
        }

        files.forEach(filename => {
            const li = document.createElement('li');
            const nameSpan = document.createElement('span');
            nameSpan.textContent = filename.replace('.json', '');
            
            const actions = document.createElement('div');
            actions.className = 'trajectory-actions';
            
            const loadBtn = document.createElement('button');
            loadBtn.textContent = 'Charger';
            loadBtn.className = 'btn-load';
            loadBtn.onclick = () => loadTrajectoryFromFile(filename);
            
            const deleteBtn = document.createElement('button');
            deleteBtn.textContent = '🗑️';
            deleteBtn.className = 'btn-delete';
            deleteBtn.onclick = () => deleteTrajectoryFile(filename);
            
            actions.appendChild(loadBtn);
            actions.appendChild(deleteBtn);
            
            li.appendChild(nameSpan);
            li.appendChild(actions);
            savedTrajectoriesList.appendChild(li);
        });
    }

    function loadTrajectoryFromFile(filename) {
        fetch(`../trajectories/${filename}`)
            .then(response => {
                if (!response.ok) throw new Error('Fichier non trouvé');
                return response.json();
            })
            .then(data => {
                // Clear current trajectory
                trajectoryPoints = [];
                pointList.innerHTML = '';
                const markers = mapWorld.querySelectorAll('.map-marker:not(.start)');
                markers.forEach(marker => marker.remove());
                
                // Load trajectory
                pointIdCounter = 0;
                if (data.trajectory && Array.isArray(data.trajectory)) {
                    data.trajectory.forEach(point => {
                        const newPoint = {
                            id: pointIdCounter++,
                            x: point.pixel.x,
                            y: point.pixel.y,
                            gps: point.gps,
                            type: point.type || 'normal',
                            photography: point.photography || 'no'
                        };
                        trajectoryPoints.push(newPoint);
                        addPointToVisualList(newPoint);
                        drawPointOnMap(newPoint);
                    });
                }
                
                // Update trajectory line
                updateTrajectoryPath();
                
                const trajName = data.meta && data.meta.name ? data.meta.name : filename.replace('.json', '');
                alert(`✅ Trajet "${trajName}" chargé (${trajectoryPoints.length} points)!`);
            })
            .catch(err => {
                console.error('Erreur chargement trajectoire:', err);
                alert('❌ Erreur lors du chargement du trajet');
            });
    }

    function deleteTrajectoryFile(filename) {
        if (!confirm(`Supprimer le trajet "${filename.replace('.json', '')}" ?`)) return;
        
        // Publier via ROS pour suppression
        const msg = new ROSLIB.Message({
            data: filename
        });
        
        deleteTrajectoryPub.publish(msg);
        console.log('Demande de suppression envoyée:', filename);
    }

    function loadForbiddenAreas() {
        // For now, just initialize empty
        // In a real application, you would load from a server or local storage
    }

    function loadBlankAreas() {
        // Load blank_area.json file
        fetch('./blank_area.json')
            .then(response => {
                if (!response.ok) {
                    throw new Error('Fichier blank_area.json introuvable');
                }
                return response.json();
            })
            .then(data => {
                // Clear existing areas
                forbiddenAreas = [];
                const rects = mapWorld.querySelectorAll('.forbidden-area');
                rects.forEach(r => r.remove());
                
                // Load areas from file
                if (data.forbiddenAreas && Array.isArray(data.forbiddenAreas)) {
                    data.forbiddenAreas.forEach(areaData => {
                        const area = {
                            x1: areaData.pixel.topLeft.x,
                            y1: areaData.pixel.topLeft.y,
                            x2: areaData.pixel.bottomRight.x,
                            y2: areaData.pixel.bottomRight.y,
                            gps1: areaData.gps.topLeft,
                            gps2: areaData.gps.bottomRight
                        };
                        forbiddenAreas.push(area);
                        drawForbiddenArea(area);
                    });
                    console.log(`${forbiddenAreas.length} zones interdites chargées depuis blank_area.json`);
                }
            })
            .catch(error => {
                console.error('Erreur lors du chargement de blank_area.json:', error);
                alert('Impossible de charger le fichier blank_area.json');
            });
    }

    function findForbiddenAreaAt(x, y) {
        // Find if a point is inside a forbidden area
        for (let i = 0; i < forbiddenAreas.length; i++) {
            const area = forbiddenAreas[i];
            if (x >= area.x1 && x <= area.x2 && y >= area.y1 && y <= area.y2) {
                return i;
            }
        }
        return -1;
    }

    function redrawForbiddenAreas() {
        // Remove all existing rectangles
        const rects = mapWorld.querySelectorAll('.forbidden-area');
        rects.forEach(r => r.remove());
        
        // Redraw all areas
        forbiddenAreas.forEach(area => drawForbiddenArea(area));
    }

    // --- Coordinate Conversion (Placeholder) ---
    function pixelToGps(x, y, imageWidth, imageHeight) {
        // This is a placeholder. You MUST replace this with a real conversion
        // based on the known GPS coordinates of the corners of your map image.
        const mapBounds = {
            topLeft: { lat: 48.86, lon: 2.35 },   // Example: Paris
            bottomRight: { lat: 48.85, lon: 2.36 }
        };
        const lat = mapBounds.topLeft.lat - (y / imageHeight) * (mapBounds.topLeft.lat - mapBounds.bottomRight.lat);
        const lon = mapBounds.topLeft.lon + (x / imageWidth) * (mapBounds.bottomRight.lon - mapBounds.topLeft.lon);
        return { lat, lon };
    }

    // --- Collapsible Sections ---
    window.toggleSection = function(sectionId) {
        const section = document.getElementById(sectionId);
        section.classList.toggle('collapsed');
        
        // Update arrow
        const header = section.previousElementSibling;
        const arrow = header.querySelector('span');
        if (arrow) {
            arrow.textContent = section.classList.contains('collapsed') ? '▶ ' + arrow.textContent.substring(2) : '▼ ' + arrow.textContent.substring(2);
        }
    };

    // Font size slider - sync with header slider
    const fontSizes = ['small', 'medium', 'large'];
    const fontSizeLabels = ['Petit', 'Moyen', 'Grand'];
    
    if (headerFontSizeSlider) {
        headerFontSizeSlider.addEventListener('input', () => {
            const value = parseInt(headerFontSizeSlider.value);
            const size = fontSizes[value - 1];
            
            // Remove all font classes
            document.body.classList.remove('font-small', 'font-medium', 'font-large');
            
            // Add selected class
            document.body.classList.add(`font-${size}`);
            
            // Update labels
            headerFontSizeLabel.textContent = fontSizeLabels[value - 1];
            
            // Save preference
            setCookie('fontSize', size, 365);
        });
    }

    // Dark mode toggle
    if (darkModeToggle) {
        darkModeToggle.addEventListener('click', () => {
            document.body.classList.toggle('dark-mode');
            const isDarkMode = document.body.classList.contains('dark-mode');
            darkModeToggle.textContent = isDarkMode ? '☀️' : '🌙';
            
            // Save preference
            setCookie('darkMode', isDarkMode ? 'on' : 'off', 365);
        });
    }

    // Load saved font size preference
    const savedFontSize = getCookie('fontSize');
    if (savedFontSize) {
        document.body.classList.add(`font-${savedFontSize}`);
        const sizeIndex = fontSizes.indexOf(savedFontSize);
        if (sizeIndex !== -1) {
            if (headerFontSizeSlider) headerFontSizeSlider.value = sizeIndex + 1;
            if (headerFontSizeLabel) headerFontSizeLabel.textContent = fontSizeLabels[sizeIndex];
        }
    } else {
        // Default to medium
        document.body.classList.add('font-medium');
        if (headerFontSizeSlider) headerFontSizeSlider.value = 2;
        if (headerFontSizeLabel) headerFontSizeLabel.textContent = 'Moyen';
    }

    // Load saved dark mode preference
    const savedDarkMode = getCookie('darkMode');
    if (savedDarkMode === 'on') {
        document.body.classList.add('dark-mode');
        if (darkModeToggle) darkModeToggle.textContent = '☀️';
    }

    clearForbiddenAreasBtn.addEventListener('click', () => {
        if (confirm('Voulez-vous vraiment effacer toutes les zones interdites ?')) {
            forbiddenAreas = [];
            const rects = mapWorld.querySelectorAll('.forbidden-area');
            rects.forEach(r => r.remove());
        }
    });

    resetForbiddenAreasBtn.addEventListener('click', () => {
        loadBlankAreas();
    });

    // Header settings button - make it rotate
    if (headerSettingsBtn) {
        headerSettingsBtn.addEventListener('mouseover', () => {
            headerSettingsBtn.style.transform = 'rotate(180deg)';
        });
        headerSettingsBtn.addEventListener('mouseout', () => {
            headerSettingsBtn.style.transform = 'rotate(0deg)';
        });
    }

    // Header toggle edit mode - sync with main button
    if (headerToggleEditModeBtn) {
        headerToggleEditModeBtn.addEventListener('click', () => {
            toggleEditModeBtn.click();
        });
    }

    // Header clear forbidden areas - sync with main button
    if (headerClearForbiddenAreasBtn) {
        headerClearForbiddenAreasBtn.addEventListener('click', () => {
            clearForbiddenAreasBtn.click();
        });
    }

    // Header reset forbidden areas - sync with main button
    if (headerResetForbiddenAreasBtn) {
        headerResetForbiddenAreasBtn.addEventListener('click', () => {
            resetForbiddenAreasBtn.click();
        });
    }

    // Update header buttons when main toggleEditModeBtn changes
    const originalToggleEditModeClick = toggleEditModeBtn.onclick;
    const updateHeaderEditMode = () => {
        if (headerToggleEditModeBtn) {
            headerToggleEditModeBtn.textContent = toggleEditModeBtn.textContent;
            headerToggleEditModeBtn.style.backgroundColor = toggleEditModeBtn.style.backgroundColor;
            headerEditButtons.style.display = editButtons.style.display;
        }
    };
    
    // Observe changes to editButtons visibility
    const observer = new MutationObserver(updateHeaderEditMode);
    observer.observe(editButtons, { attributes: true, attributeFilter: ['style'] });
});

// =======================================================================
// FONCTIONS GLOBALES
// =======================================================================

function toggleHeaderSettings() {
    const modal = document.getElementById('header-settings-modal');
    if (modal) {
        modal.style.display = (modal.style.display === 'block') ? 'none' : 'block';
    }
}

// Fermer la modale quand on clique en dehors
document.addEventListener('click', (e) => {
    const modal = document.getElementById('header-settings-modal');
    if (modal && e.target === modal) {
        modal.style.display = 'none';
    }
});

// Charger les préférences au démarrage
window.addEventListener('load', () => {
    const fontSizeSlider = document.getElementById('header-font-size-slider');
    const fontSizeLabels = ['Petit', 'Moyen', 'Grand'];
    const fontSizes = ['small', 'medium', 'large'];
    
    const savedFontSize = getCookie('fontSize');
    if (savedFontSize && fontSizeSlider) {
        const sizeIndex = fontSizes.indexOf(savedFontSize);
        if (sizeIndex !== -1) {
            fontSizeSlider.value = sizeIndex + 1;
        }
    }
    
    const darkMode = getCookie('darkMode');
    if (darkMode === 'on') {
        document.body.classList.add('dark-mode');
        const btn = document.getElementById('dark-mode-toggle');
        if (btn) {
            btn.textContent = '☀️';
        }
    }
});

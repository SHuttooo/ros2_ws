document.addEventListener('DOMContentLoaded', () => {
    // --- DOM Elements ---
    const mapContainer = document.getElementById('map-container');
    const mapWorld = document.getElementById('map-world');
    const mapImage = document.getElementById('map-image');
    const trajectoryPath = document.getElementById('trajectory-path');
    const gpsCoordsDisplay = document.getElementById('gps-coords');
    const pointList = document.getElementById('point-list');
    const sendTrajectoryBtn = document.getElementById('send-trajectory');
    const clearTrajectoryBtn = document.getElementById('clear-trajectory');
    const removeLastPointBtn = document.getElementById('remove-last-point');
    const toggleEditModeBtn = document.getElementById('toggle-edit-mode');
    const clearForbiddenAreasBtn = document.getElementById('clear-forbidden-areas');
    const resetForbiddenAreasBtn = document.getElementById('reset-forbidden-areas');
    const downloadDataBtn = document.getElementById('download-data');
    const trajectoryButtons = document.getElementById('trajectory-buttons');
    const editButtons = document.getElementById('edit-buttons');
    const savedTrajectoriesList = document.getElementById('saved-trajectories-list');
    const zoomInBtn = document.getElementById('zoom-in');
    const zoomOutBtn = document.getElementById('zoom-out');
    const settingsBtn = document.getElementById('settings-btn');
    const settingsMenu = document.getElementById('settings-menu');
    const fontSizeSlider = document.getElementById('font-size-slider');
    const fontSizeLabel = document.getElementById('font-size-label');
    const darkModeToggle = document.getElementById('dark-mode-toggle');

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
        loadSavedTrajectories();
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
    sendTrajectoryBtn.addEventListener('click', () => {
        if (trajectoryPoints.length === 0) {
            alert('Le trajet est vide.');
            return;
        }
        const poseArray = new ROSLIB.Message({
            header: { stamp: new Date().toISOString(), frame_id: 'map' },
            poses: trajectoryPoints.map(p => ({
                position: { x: p.gps.lon, y: p.gps.lat, z: 0 },
                orientation: { x: 0, y: 0, z: 0, w: 1 }
            }))
        });
        trajectoryTopic.publish(poseArray);
        console.log('Trajet envoyé:', poseArray);
        alert('Trajet envoyé à ROS !');
    });

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

    downloadDataBtn.addEventListener('click', () => {
        downloadAllData();
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

    function downloadAllData() {
        // Create a single JSON file with all data
        const data = {
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
        
        // Create and download JSON file
        const jsonContent = JSON.stringify(data, null, 2);
        const blob = new Blob([jsonContent], { type: 'application/json;charset=utf-8;' });
        const link = document.createElement('a');
        link.href = URL.createObjectURL(blob);
        const timestamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, -5);
        link.download = `user_models/navigation_data_${timestamp}.json`;
        link.click();
        URL.revokeObjectURL(link.href);
        
        console.log('Données téléchargées dans user_models/');
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

    // --- Cookie Management ---
    window.saveTrajectoryToCookies = function() {
        if (trajectoryPoints.length === 0) {
            alert('Aucun point dans le trajet.');
            return;
        }

        const name = prompt('Nom du trajet:');
        if (!name) return;

        const trajectoryData = {
            name: name,
            timestamp: new Date().toISOString(),
            startPoint: startPoint,
            points: trajectoryPoints
        };

        // Get existing trajectories
        let savedTrajectories = [];
        const cookieData = getCookie('savedTrajectories');
        if (cookieData) {
            try {
                savedTrajectories = JSON.parse(cookieData);
            } catch (e) {
                console.error('Error parsing saved trajectories:', e);
            }
        }

        // Add new trajectory
        savedTrajectories.push(trajectoryData);

        // Save to cookie (max 4KB per cookie)
        setCookie('savedTrajectories', JSON.stringify(savedTrajectories), 365);
        
        alert('Trajet sauvegardé!');
        loadSavedTrajectories();
    };

    function loadSavedTrajectories() {
        savedTrajectoriesList.innerHTML = '';
        
        const cookieData = getCookie('savedTrajectories');
        if (!cookieData) return;

        try {
            const savedTrajectories = JSON.parse(cookieData);
            savedTrajectories.forEach((traj, index) => {
                const li = document.createElement('li');
                const nameSpan = document.createElement('span');
                nameSpan.textContent = `${traj.name} (${traj.points.length} pts)`;
                
                const actions = document.createElement('div');
                actions.className = 'trajectory-actions';
                
                const loadBtn = document.createElement('button');
                loadBtn.textContent = 'Charger';
                loadBtn.className = 'btn-load';
                loadBtn.onclick = () => loadTrajectory(index);
                
                const deleteBtn = document.createElement('button');
                deleteBtn.textContent = '🗑️';
                deleteBtn.className = 'btn-delete';
                deleteBtn.onclick = () => deleteTrajectory(index);
                
                actions.appendChild(loadBtn);
                actions.appendChild(deleteBtn);
                
                li.appendChild(nameSpan);
                li.appendChild(actions);
                savedTrajectoriesList.appendChild(li);
            });
        } catch (e) {
            console.error('Error loading saved trajectories:', e);
        }
    }

    function loadTrajectory(index) {
        const cookieData = getCookie('savedTrajectories');
        if (!cookieData) return;

        try {
            const savedTrajectories = JSON.parse(cookieData);
            const traj = savedTrajectories[index];
            
            // Clear current trajectory
            trajectoryPoints = [];
            pointList.innerHTML = '';
            const markers = mapWorld.querySelectorAll('.map-marker:not(.start)');
            markers.forEach(marker => marker.remove());
            
            // Load trajectory
            pointIdCounter = 0;
            traj.points.forEach(point => {
                const newPoint = { ...point, id: pointIdCounter++ };
                trajectoryPoints.push(newPoint);
                addPointToVisualList(newPoint);
                drawPointOnMap(newPoint);
            });
            
            alert(`Trajet "${traj.name}" chargé!`);
        } catch (e) {
            console.error('Error loading trajectory:', e);
        }
    }

    function deleteTrajectory(index) {
        if (!confirm('Supprimer ce trajet?')) return;
        
        const cookieData = getCookie('savedTrajectories');
        if (!cookieData) return;

        try {
            const savedTrajectories = JSON.parse(cookieData);
            savedTrajectories.splice(index, 1);
            setCookie('savedTrajectories', JSON.stringify(savedTrajectories), 365);
            loadSavedTrajectories();
        } catch (e) {
            console.error('Error deleting trajectory:', e);
        }
    }

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

    // --- Settings Menu ---
    settingsBtn.addEventListener('click', (e) => {
        e.stopPropagation();
        settingsMenu.style.display = settingsMenu.style.display === 'none' ? 'block' : 'none';
    });

    // Close settings menu when clicking outside
    document.addEventListener('click', (e) => {
        if (!settingsMenu.contains(e.target) && e.target !== settingsBtn) {
            settingsMenu.style.display = 'none';
        }
    });

    // Font size slider
    const fontSizes = ['small', 'medium', 'large'];
    const fontSizeLabels = ['Petit', 'Moyen', 'Grand'];
    
    fontSizeSlider.addEventListener('input', () => {
        const value = parseInt(fontSizeSlider.value);
        const size = fontSizes[value - 1];
        
        // Remove all font classes
        document.body.classList.remove('font-small', 'font-medium', 'font-large');
        
        // Add selected class
        document.body.classList.add(`font-${size}`);
        
        // Update label
        fontSizeLabel.textContent = fontSizeLabels[value - 1];
        
        // Save preference
        setCookie('fontSize', size, 365);
    });

    // Dark mode toggle
    darkModeToggle.addEventListener('click', () => {
        document.body.classList.toggle('dark-mode');
        const isDarkMode = document.body.classList.contains('dark-mode');
        darkModeToggle.textContent = isDarkMode ? 'ON' : 'OFF';
        darkModeToggle.classList.toggle('active', isDarkMode);
        
        // Save preference
        setCookie('darkMode', isDarkMode ? 'on' : 'off', 365);
    });

    // Load saved font size preference
    const savedFontSize = getCookie('fontSize');
    if (savedFontSize) {
        document.body.classList.add(`font-${savedFontSize}`);
        const sizeIndex = fontSizes.indexOf(savedFontSize);
        if (sizeIndex !== -1) {
            fontSizeSlider.value = sizeIndex + 1;
            fontSizeLabel.textContent = fontSizeLabels[sizeIndex];
        }
    } else {
        // Default to medium
        document.body.classList.add('font-medium');
        fontSizeSlider.value = 2;
        fontSizeLabel.textContent = 'Moyen';
    }

    // Load saved dark mode preference
    const savedDarkMode = getCookie('darkMode');
    if (savedDarkMode === 'on') {
        document.body.classList.add('dark-mode');
        darkModeToggle.textContent = 'ON';
        darkModeToggle.classList.add('active');
    }
});

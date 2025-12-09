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
    const zoomInBtn = document.getElementById('zoom-in');
    const zoomOutBtn = document.getElementById('zoom-out');

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
    
    // Fixed start point (example coordinates, adjust as needed)
    const startPoint = { x: 100, y: 100, gps: { lat: 0, lon: 0 }, type: 'start' }; 

    // --- Map Setup ---
    mapImage.src = './map.jpg'; // TODO: Replace with your map
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
        // Ignore clicks on buttons
        if (event.target.tagName === 'BUTTON') return;
        
        if (isDragging) return;
        
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
});

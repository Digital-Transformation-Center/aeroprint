import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.176.0/build/three.module.js';
import { io } from 'https://cdn.jsdelivr.net/npm/socket.io-client@4.7.5/dist/socket.io.esm.min.js';

const show_controls = false;

const controls = document.getElementById('controls');
const startingHeightContainer = document.getElementById('startingHeightContainer');
const radius_slider = document.getElementById('radius');
const height_slider = document.getElementById('height');
const turns_slider = document.getElementById('turns');
const startHeightSlider = document.getElementById('startHeightSlider');
const startHeightInput = document.getElementById('startHeight');
const saveButton = document.getElementById('save-button');

if (show_controls) {
    // Show controls
    if (controls) controls.style.display = '';
    if (startingHeightContainer) startingHeightContainer.style.display = 'flex';
    if (radius_slider) radius_slider.disabled = false;
    if (height_slider) height_slider.disabled = false;
    if (turns_slider) turns_slider.disabled = false;
    if (startHeightSlider) startHeightSlider.disabled = false;
    if (startHeightInput) startHeightInput.disabled = false;
    if (saveButton) saveButton.style.display = '';
    console.log('Controls are enabled');
} else {
    // Hide controls and disable inputs
    if (controls) controls.style.display = 'none';
    else console.warn('Controls element not found');
    if (startingHeightContainer) startingHeightContainer.style.display = 'none';
    else console.warn('Starting height container not found');
    if (radius_slider) radius_slider.disabled = true;
    if (height_slider) height_slider.disabled = true;
    if (turns_slider) turns_slider.disabled = true;
    if (startHeightSlider) startHeightSlider.disabled = true;
    if (startHeightInput) startHeightInput.disabled = true;
    if (saveButton) saveButton.style.display = 'none';
    console.log('Controls are disabled');
}

console.log('scene.js loaded');
window.socket = io();
// Listen for notifications from the server specific to flight_config
window.socket.on('flight_config_notification', (data) => {
    console.log('[flight_config] Notification received:', data);
    // Optionally, display in the UI or trigger a visual effect here
    // For example, flash the cylinder or show a message
    // You can expand this as needed for your application
});
let scene, camera, renderer, cylinderMesh, helixLine, axesGroup;
let placeHereMarker, placeHereLabel;
let pulseIndex = 0;
let pulseDirection = -1;
let pulseLine;


let flight_path_radius = 0.0;
let flight_path_height = 0.0;
let flight_path_turns = 0;
let flight_path_start_height = 0.0;
let path_did_change = false;
let max_axis_length = 15; // Maximum length for axes markers
let danger_radius = 0;
let danger_height = 0;

const metric_slider_limits = {
    radius: { min: 0.1, max: 1, steps: 0.05, currentValue: 0.2 },
    height: { min: 0.1, max: 2, steps: 0.05, currentValue: 0.3 },
    turns: { min: 1, max: 10, steps: 1, currentValue: 2 }
};

const imperial_slider_limits = {
    radius: { min: 0.3, max: 3.5, steps: 0.1, currentValue: 2 },
    height: { min: 0.3, max: 6.5, steps: 0.1, currentValue: 2 },
    turns: { min: 1, max: 10, steps: 1, currentValue: 2 }
};

export function updateSliderLimits(units) {
    const limits = units === 'feet' ? imperial_slider_limits : metric_slider_limits;
    updateSlider(radius_slider, limits.radius);
    updateSlider(height_slider, limits.height);
    updateSlider(turns_slider, limits.turns);
}

function updateSlider(slider, { min, max, steps, currentValue }) {
    slider.dispatchEvent(new Event('input'));
    slider.min = min;
    slider.max = max;
    slider.value = currentValue;
    slider.step = steps;
    document.getElementById(slider.id + 'Value').textContent = slider.value;
}

export function createTextSprite(text, color = "#fff") {
    const scale = 0.5;
    const baseWidth = 256, baseHeight = 96;
    const canvas = document.createElement('canvas');
    canvas.width = baseWidth * scale;
    canvas.height = baseHeight * scale;
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.font = `bold ${64 * scale}px Arial, Helvetica, sans-serif`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.shadowColor = "#222";
    ctx.shadowBlur = 16 * scale;
    ctx.fillStyle = color;
    ctx.fillText(text, canvas.width / 2, canvas.height / 2);
    ctx.shadowBlur = 0;
    ctx.lineWidth = 0 * scale;
    ctx.strokeStyle = "#222";
    ctx.strokeText(text, canvas.width / 2, canvas.height / 2);
    const texture = new THREE.CanvasTexture(canvas);
    texture.needsUpdate = true;
    texture.minFilter = THREE.LinearFilter;
    texture.generateMipmaps = false;
    const spriteMaterial = new THREE.SpriteMaterial({ map: texture, transparent: true });
    const sprite = new THREE.Sprite(spriteMaterial);
    sprite.scale.set(3, 1, 1);
    return sprite;
}

export function createAxesMarkers(length = 5, step = 1) {
    const group = new THREE.Group();
    const axes = [
        { dir: [1,0,0], color: 0xffffff, label: 'X' },
        { dir: [0,1,0], color: 0x00a4ff, label: 'Z' },
        { dir: [0,0,1], color: 0xffffff, label: 'Y' }
    ];
    axes.forEach(axis => {
        const mat = new THREE.LineBasicMaterial({ color: axis.color });
        const geo = new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(0,0,0),
            new THREE.Vector3(axis.dir[0]*length, axis.dir[1]*length, axis.dir[2]*length)
        ]);
        group.add(new THREE.Line(geo, mat));
        for (let i = 1; i <= length; i += step) {
            const markerGeo = new THREE.SphereGeometry(0.07, 8, 8);
            const markerMat = new THREE.MeshBasicMaterial({ color: axis.color });
            const marker = new THREE.Mesh(markerGeo, markerMat);
            marker.position.set(axis.dir[0]*i, axis.dir[1]*i, axis.dir[2]*i);
            group.add(marker);
        }
    });
    return group;
}



export function createHelixGeometry(radius, pitch, turns, points, baseY = 0, minY = 0.1) {
    const geometry = new THREE.BufferGeometry();
    const positions = [];
    const totalAngle = turns * 2 * Math.PI;
    for (let i = 0; i <= points; i++) {
        const t = i / points;
        const angle = t * totalAngle;
        const x = radius * Math.cos(angle);
        const z = radius * Math.sin(angle);
        let y = baseY + pitch * turns * t;
        if (y < minY) y = minY;
        positions.push(x, y, z);
    }
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    return geometry;
}

export function updateScene() {
    const radius = parseFloat(document.getElementById('radius').value);
    const height = parseFloat(document.getElementById('height').value);
    const turns = parseInt(document.getElementById('turns').value);
    const startHeight = parseFloat(document.getElementById('startHeight').value);
    const helixRadius = radius + 0.3;
    let points = 300;
    const minHelixY = 0.2;
    // Compute helix length and adjust number of points

    if (cylinderMesh) scene.remove(cylinderMesh);
    if (helixLine) scene.remove(helixLine);
    if (pulseLine) scene.remove(pulseLine);
    if (placeHereMarker) scene.remove(placeHereMarker);
    if (placeHereLabel) scene.remove(placeHereLabel);

    // Cylinder
    const geometry = new THREE.CylinderGeometry(radius, radius, height, 64);
    const material = new THREE.MeshStandardMaterial({ color: 0x2196f3, transparent: true, opacity: 0.8 });
    cylinderMesh = new THREE.Mesh(geometry, material);
    cylinderMesh.position.set(0, startHeight + height/2, 0);
    scene.add(cylinderMesh);

    // Helix
    const pitch = height / turns;
    const helixStart = Math.max(startHeight, minHelixY);

    const helixLength = turns * Math.sqrt((2 * Math.PI * helixRadius) ** 2 + pitch ** 2);
    const pointsPerUnit = 10; // adjust for smoothness

    // Update flight_path variables
    flight_path_radius = helixRadius;
    flight_path_height = height;
    flight_path_turns = turns;
    flight_path_start_height = startHeight;
    path_did_change = true;

    points = Math.max(20, Math.floor(helixLength * pointsPerUnit));

    const geometryHelix = createHelixGeometry(helixRadius, pitch, turns, points, helixStart, minHelixY);
    const materialHelix = new THREE.LineBasicMaterial({ color: 0xff2222 });
    helixLine = new THREE.Line(geometryHelix, materialHelix);
    scene.add(helixLine);

    // Place Here marker and label (at base of cylinder)
    placeHereMarker = new THREE.Mesh(
        new THREE.SphereGeometry(0.11, 16, 16),
        new THREE.MeshBasicMaterial({ color: 0x53cc3d })
    );
    placeHereMarker.position.set(0, startHeight, 0);
    scene.add(placeHereMarker);
    placeHereLabel = createTextSprite("Object", "#53cc3d");
    placeHereLabel.position.set(0, startHeight + 0.5, 0);
    scene.add(placeHereLabel);

    // Pulse line setup (optional, for animation)
    pulseIndex = 0;
    if (pulseLine) scene.remove(pulseLine);
    const pulseGeo = new THREE.BufferGeometry();
    const positions = geometryHelix.getAttribute('position');
    const pulsePositions = [];
    for (let i = 0; i <= pulseIndex; i++) {
        pulsePositions.push(positions.getX(i), positions.getY(i), positions.getZ(i));
    }
    pulseGeo.setAttribute('position', new THREE.Float32BufferAttribute(pulsePositions, 3));
    const pulseMat = new THREE.LineBasicMaterial({ color: 0x2266ff, linewidth: 5 });
    pulseLine = new THREE.Line(pulseGeo, pulseMat);
    scene.add(pulseLine);
    pulseLine._helixPositions = positions;
    pulseLine._helixPoints = points;
}



export function addInputListeners() {
    if (!show_controls) return; // Only add listeners if controls are enabled
    ["radius", "height", "turns"].forEach(id => {
        // document.getElementById(id).addEventListener("input", updateScene);
        // document.getElementById(id + 'Value').textContent = document.getElementById(id).value;
        // console.log('Input listener added for:', id);
        document.getElementById(id).addEventListener("input", (event) => {
            window.socket.emit('path_values_changed', { [id]: event.target.value });
            const value = event.target.value;
            document.getElementById(id + 'Value').textContent = value;
            updateScene();
        });
    });
    // Sync slider and number input for startHeight
    const slider = document.getElementById("startHeightSlider");
    const number = document.getElementById("startHeight");
    slider.addEventListener("input", () => {
        number.value = slider.value;
        updateScene();
    });
    number.addEventListener("input", () => {
        slider.value = number.value;
        updateScene();
    });
    document.getElementById("save-button").addEventListener("click", () => {
        console.log('Save button clicked');
        if (window.socket && window.socket.emit) {
            if (path_did_change) {
                const params = getParameters();
                console.log('Parameters to save:', params);
                window.socket.emit('save_flight_config', params);
            } else {
                window.socket.emit('notification', {'message': 'No changes to save', 'warning': 'bad'});
            }
        } else {
            console.error('Socket not initialized or emit function not available');
        }
    });
}

if (!show_controls) {
    // Listen for socket data to update scene when controls are hidden
    window.socket.on('path_values_changed', (data) => {
        console.log('Received path_values_changed:', data);
        // Update sliders and scene from socket data
        if (typeof data.radius !== 'undefined') {
            radius_slider.value = data.radius;
            document.getElementById('radiusValue').textContent = data.radius;
        }
        if (typeof data.height !== 'undefined') {
            height_slider.value = data.height;
            document.getElementById('heightValue').textContent = data.height;
        }
        if (typeof data.turns !== 'undefined') {
            turns_slider.value = data.turns;
            document.getElementById('turnsValue').textContent = data.turns;
        }
        if (typeof data.startHeight !== 'undefined') {
            startHeightInput.value = data.startHeight;
            startHeightSlider.value = data.startHeight;
        }
        updateScene();
    });
}

const unitLabel = document.getElementById('unitLabel');
unitLabel.textContent = window.currentUnits || 'meters';

window.socket.on('units_changed', (data) => {
    console.log('Received units_changed:', data);
    window.currentUnits = data.units;
    updateSliderLimits(window.currentUnits);
    unitLabel.textContent = window.currentUnits;
});

export function getParameters() {
    if (window.currentUnits == 'meters') {
        danger_radius = Math.ceil((flight_path_radius + 0.3) * 10) / 10;
        danger_height = Math.ceil((flight_path_height + 0.3) * 10) / 10;
        window.socket.emit('notification', { message: 'Path saved. Ensure space is at least ' + danger_radius + ' meters in radius and ' + danger_height + ' meters in height.', warning: 'bad' });
        return {
            radius: flight_path_radius,
            height: flight_path_height,
            turns: flight_path_turns,
            startHeight: flight_path_start_height,
        };
    } else if (window.currentUnits == 'feet') {
        danger_radius = Math.ceil((flight_path_radius + 1) * 10) / 10;
        danger_height = Math.ceil((flight_path_height + 1) * 10) / 10;
        window.socket.emit('notification', { message: 'Path saved. Ensure space is at least ' + danger_radius + ' feet in radius and ' + danger_height + ' feet in height.', warning: 'bad' });
        return {
            radius: flight_path_radius / 3.28084,
            height: flight_path_height / 3.28084,
            turns: flight_path_turns,
            startHeight: flight_path_start_height / 3.28084,
        };
    }
    console.error('Unknown units:', window.currentUnits);
}

export function animate() {
    requestAnimationFrame(animate);
    // Animate pulse along the helix
    if (pulseLine && pulseLine._helixPositions) {
        const points = pulseLine._helixPoints;
        pulseIndex += pulseDirection * 0.5;
        if (pulseIndex >= points) {
            pulseIndex = points;
            pulseDirection = 1;
        } else if (pulseIndex <= 0) {
            pulseIndex = points - 1;
            pulseDirection = -1;
        }
        const pulsePositions = [];
        const startIdx = Math.max(0, Math.floor(pulseIndex) - 3);
        const endIdx = Math.floor(pulseIndex);
        for (let i = startIdx; i <= endIdx; i++) {
            pulsePositions.push(
                pulseLine._helixPositions.getX(i),
                pulseLine._helixPositions.getY(i),
                pulseLine._helixPositions.getZ(i)
            );
        }
        pulseLine.geometry.setAttribute(
            'position',
            new THREE.Float32BufferAttribute(pulsePositions, 3)
        );
        pulseLine.geometry.setDrawRange(0, pulseIndex + 1);
        pulseLine.geometry.attributes.position.needsUpdate = true;
    }
    renderer.render(scene, camera);
}

export function init() {
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x222222);
    const width = Math.min(window.innerWidth * 0.9, 800);
    const heightPx = 500;
    camera = new THREE.PerspectiveCamera(20, width / heightPx, 0.1, 100);
    let cameraDistance = 10;
    let theta = Math.PI/4, phi = Math.PI/6;
    function updateCameraPosition() {
        camera.position.x = cameraDistance * Math.sin(theta) * Math.cos(phi);
        camera.position.y = cameraDistance * Math.sin(phi) + 2;
        camera.position.z = cameraDistance * Math.cos(theta) * Math.cos(phi);
        camera.lookAt(0, 1, 0);
        // Scale label
        const baseScale = 3;
        const scale = baseScale * (cameraDistance / 10);
        if (placeHereLabel) placeHereLabel.scale.set(scale, scale / 3, 1);
    }
    camera.position.set(6, 6, 6);
    camera.lookAt(0, 2, 0);
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(width, heightPx);
    renderer.setClearColor(0x888888);
    renderer.domElement.style.display = "block";
    renderer.domElement.style.margin = "0 auto";
    renderer.domElement.style.position = "relative";
    renderer.domElement.style.left = "50%";
    renderer.domElement.style.transform = "translateX(-50%)";
    document.getElementById('container').appendChild(renderer.domElement);
    // Lighting
    const ambient = new THREE.AmbientLight(0xffffff, 0.7);
    scene.add(ambient);
    const directional = new THREE.DirectionalLight(0xffffff, 0.8);
    directional.position.set(10, 10, 10);
    scene.add(directional);
    axesGroup = createAxesMarkers(max_axis_length, 1);
    scene.add(axesGroup);
    updateScene();
    // Orbit controls
    let isDragging = false, prevX, prevY;
    const isTouchDevice = 'ontouchstart' in window || navigator.maxTouchPoints > 0;
    if (isTouchDevice) {
        renderer.domElement.addEventListener('touchstart', (e) => {
            const touch = e.changedTouches[0];
            const x = touch.clientX;
            const y = touch.clientY;
            isDragging = true;
            prevX = x; prevY = y;
        });

        
        renderer.domElement.addEventListener('touchend', () => {
            isDragging = false;
        });
        let initialPinchDistance = null;
        let initialCameraDistance = cameraDistance;

        renderer.domElement.addEventListener('touchmove', (e) => {
            if (e.touches.length === 2) {
                e.preventDefault();
                const dx = e.touches[0].clientX - e.touches[1].clientX;
                const dy = e.touches[0].clientY - e.touches[1].clientY;
                const distance = Math.sqrt(dx * dx + dy * dy);
                if (initialPinchDistance === null) {
                    initialPinchDistance = distance;
                    initialCameraDistance = cameraDistance;
                } else {
                    const scale = initialPinchDistance / distance;
                    cameraDistance = Math.max(3, Math.min(30, initialCameraDistance * scale));
                    updateCameraPosition();
                }
            }
            else if (e.touches.length < 2) {
                initialPinchDistance = null; // Reset pinch distance when only one finger is on screen
                if (!isDragging) return;
                const touch = e.changedTouches[0];
                const x = touch.clientX;
                const y = touch.clientY;
                theta -= (x - prevX) * 0.01;
                phi += (y - prevY) * 0.01;
                phi = Math.max(-Math.PI/2 + 0.1, Math.min(Math.PI/2 - 0.1, phi));
                updateCameraPosition();
                prevX = x; prevY = y;
            }
        }, { passive: false });

        renderer.domElement.addEventListener('touchend', (e) => {
            if (e.touches.length < 2) {
                initialPinchDistance = null;
            }
        });

    }
    renderer.domElement.addEventListener('mousedown', e => {
        isDragging = true; prevX = e.clientX; prevY = e.clientY;
        console.log('Mouse down at:', prevX, prevY);
    });
    window.addEventListener('mousemove', e => {
        if (!isDragging) return;
        theta -= (e.clientX - prevX) * 0.01;
        phi -= (e.clientY - prevY) * 0.01;
        phi = Math.max(-Math.PI/2 + 0.1, Math.min(Math.PI/2 - 0.1, phi));
        updateCameraPosition();
        prevX = e.clientX; prevY = e.clientY;
    });
    window.addEventListener('mouseup', () => { isDragging = false; });
    renderer.domElement.addEventListener('wheel', (e) => {
        e.preventDefault();
        cameraDistance += e.deltaY * 0.01;
        cameraDistance = Math.max(3, Math.min(30, cameraDistance));
        updateCameraPosition();
    }, { passive: false });
    updateCameraPosition();
    addInputListeners();
    animate();
}
window.addEventListener('resize', () => {
    const width = Math.min(window.innerWidth * 0.9, 800);
    const heightPx = 500;
    camera.aspect = width / heightPx;
    camera.updateProjectionMatrix();
    renderer.setSize(width, heightPx);
});

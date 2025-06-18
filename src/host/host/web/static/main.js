// Shutdown button logic
const shutdownBtn = document.getElementById('shutdownBtn');
shutdownBtn.onclick = () => {
    // window.socket.emit('shutdown_test_node');
    land_flight();
};
const openBtn = document.getElementById('openModalBtn');
const closeBtn = document.getElementById('closeModalBtn');
const modalInfoBtn = document.getElementById('modalInquiryBtn');
modalInfoBtn.onclick = () => {
    alert('Use this visualizer to configure the flight path and object parameters.\n' +
            'The blue cylinder is the area in which objects will be digitized. \n' +
            'The red line is the flight path of the drone.\n\n' +
            '1. Adjust the radius, height, and number of passes using the sliders.\n' +
            '2. The vertical slider sets the starting height of the flight path.\n' +
            '3. Click "Save" to apply your settings.\n\n' +
            'For more information, refer to the documentation.');
};
const modal = document.getElementById('flightModal');
const notif_bg_default = '#323232';
openBtn.onclick = () => { modal.classList.add('active'); };
closeBtn.onclick = () => { modal.classList.remove('active'); };
window.onclick = (e) => {
    if (e.target === modal) modal.classList.remove('active');
};
window.onkeydown = (e) => {
    if (e.key === "Escape") modal.classList.remove('active');
};

import { io } from 'https://cdn.jsdelivr.net/npm/socket.io-client@4.7.5/dist/socket.io.esm.min.js';
window.socket = io();
// Heartbeat indicator logic
const heartbeatDot = document.getElementById('heartbeatDot');
const heartbeatText = document.getElementById('heartbeatText');
let heartbeatTimeout = null;
function setHeartbeatStatus(connected) {
    if (connected) {
        heartbeatDot.classList.remove('disconnected');
        heartbeatDot.classList.add('connected');
        heartbeatText.textContent = 'Connected';
    } else {
        heartbeatDot.classList.remove('connected');
        heartbeatDot.classList.add('disconnected');
        heartbeatText.textContent = 'Disconnected';
    }
}
window.socket.on('heartbeat', data => {
    setHeartbeatStatus(data && data.ok);
    if (heartbeatTimeout) clearTimeout(heartbeatTimeout);
    // If no heartbeat in 3s, set to disconnected
    heartbeatTimeout = setTimeout(() => setHeartbeatStatus(false), 3000);
});
// --- Notification listener ---
window.socket.on('notification', data => {
    let msg = data && data.message ? data.message : String(data);
    let stat = data.warning;
    notify(msg, stat);
});

function notify(message, stat = 'default') {
    const notif = document.getElementById('notification');
    notif.textContent = message || 'Notification';
    if (stat == 'good') {
        notif.style.backgroundColor = '#2ecc40';
    } else if (stat == 'bad') {
        notif.style.backgroundColor = '#e53935';
    } else {
        notif.style.backgroundColor = notif_bg_default;
    }
    notif.style.display = 'block';
    notif.style.opacity = '1';
    // Hide after 3 seconds
    setTimeout(() => {
        notif.style.transition = 'opacity 0.5s';
        notif.style.opacity = '0';
        setTimeout(() => { notif.style.display = 'none'; notif.style.transition = ''; }, 500);
    }, 3000);
}
    
const slider = document.getElementById('flightSlider');
const knob = document.getElementById('sliderKnob');
knob.classList.add('warn');
const knobText = knob.querySelector('.knob-text');
const arrow = document.getElementById('sliderArrow');

arrow.innerHTML = `
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
        <path d="M5 12h14M13 6l6 6-6 6" stroke="white" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
        </svg>
    `;
// arrow.innerHTML = '';
const knobWidth = 96;
const sliderWidth = 320;
const minLeft = 4;
const maxLeft = sliderWidth - knobWidth - 4;
let dragging = false;
let startX = 0;
let knobStartLeft = 0;
let startCondition = false; // false = not started, true = started
let knob_status = 1;

// import { io } from 'https://cdn.jsdelivr.net/npm/socket.io-client@4.7.5/dist/socket.io.esm.min.js';
// window.socket = io();


function setKnobState() {
    set_knob_appearance();
    if (!startCondition && knob_status == 0) {
    land_flight();
    // let content = knob_status == 0 ? 'Start' : 'Not Ready';
    // knob.querySelector('.knob-text').textContent = content;
    // knob.classList.remove('active');
    // Right arrow SVG
    arrow.innerHTML = `
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
        <path d="M5 12h14M13 6l6 6-6 6" stroke="white" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
        </svg>
    `;
    arrow.style.left = '';
    arrow.style.right = '12px';
    } else if (knob_status == 0) {
    start_flight();
    knob.querySelector('.knob-text').textContent = 'Active';
    knob.classList.add('active');
    // Left arrow SVG
    arrow.innerHTML = `
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
        <path d="M19 12H5M11 6l-6 6 6 6" stroke="white" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
        </svg>
    `;
    arrow.style.right = '';
    arrow.style.left = '12px';
    }
}

function set_knob_appearance() {
    if (startCondition) {
    knob.classList.add('active');
    knob.querySelector('.knob-text').textContent = 'Land';
    knob.classList.remove('warn');
    knob.classList.add('active');
    arrow.style.left = '12px';
    arrow.style.right = '';
    } else {
    knob.classList.remove('active');
    if (knob_status == 0) {
        knob.querySelector('.knob-text').textContent = 'Start';
        knob.classList.remove('warn');
    } else {
        knob.querySelector('.knob-text').textContent = 'Warn';
        knob.classList.add('warn');
    }
    arrow.style.left = '';
    arrow.style.right = '12px';
    }
}

function start_flight() {
    // Emit the flight start event
    console.log('Starting flight...');
//   window.socket.emit('flight_start', { start: true });
    window.socket.emit('start_flight');
    notify('Requesting flight start...', 'good');
    // window.socket.emit('notification', { message: 'Requesting flight start...', warning: 'good' });
}

function land_flight() {
    // Emit the flight land event
    console.log('Landing flight...');
    window.socket.emit('land_flight');
    notify('Requesting flight land...', 'good');
}

function setKnobPosition(pos, animate = true) {
    knob.style.transition = animate ? 'left 0.2s cubic-bezier(.4,2,.6,1), background 0.2s' : 'none';
    knob.style.left = `${pos}px`;
}

function bounceKnob() {
    if (!startCondition) {
    setKnobPosition(minLeft);
    } else {
    setKnobPosition(maxLeft);
    }
}

function onDragStart(e) {
    dragging = true;
    startX = e.type.startsWith('touch') ? e.touches[0].clientX : e.clientX;
    knobStartLeft = parseInt(knob.style.left, 10) || minLeft;
    knob.style.transition = 'none';
    document.addEventListener('mousemove', onDragMove);
    document.addEventListener('mouseup', onDragEnd);
    document.addEventListener('touchmove', onDragMove, {passive: false});
    document.addEventListener('touchend', onDragEnd);
}

function onDragMove(e) {
    if (!dragging) return;
    e.preventDefault();
    const clientX = e.type.startsWith('touch') ? e.touches[0].clientX : e.clientX;
    let dx = clientX - startX;
    let newLeft = knobStartLeft + dx;
    newLeft = Math.max(minLeft, Math.min(maxLeft, newLeft));
    setKnobPosition(newLeft, false);
}

function onDragEnd(e) {
    if (!dragging) return;
    dragging = false;
    const left = parseInt(knob.style.left, 10) || minLeft;
    if (knob_status == 1) {
        startCondition = false;
        setKnobState();
        setKnobPosition(minLeft);
    }
    else if (!startCondition) {
    if (left > maxLeft - 10) {
        // Dragged all the way right: activate start
        startCondition = true;
        setKnobState();
        setKnobPosition(maxLeft);
    } else {
        // Not far enough: bounce back left
        setKnobPosition(minLeft);
    }
    } else {
    if (left < minLeft + 10) {
        // Dragged all the way left: deactivate start
        startCondition = false;
        setKnobState();
        setKnobPosition(minLeft);
    } else{
        // Not far enough: bounce back right
        setKnobPosition(maxLeft);
    }
    }
    document.removeEventListener('mousemove', onDragMove);
    document.removeEventListener('mouseup', onDragEnd);
    document.removeEventListener('touchmove', onDragMove);
    document.removeEventListener('touchend', onDragEnd);
}

knob.addEventListener('mousedown', onDragStart);
knob.addEventListener('touchstart', onDragStart, {passive: false});

// On hover, show correct arrow

// --- Warn tooltip logic ---
// Create tooltip element
const warnTooltip = document.createElement('div');
warnTooltip.style.position = 'fixed';
warnTooltip.style.background = '#e53935';
warnTooltip.style.color = '#fff';
warnTooltip.style.padding = '8px 16px';
warnTooltip.style.borderRadius = '8px';
warnTooltip.style.fontSize = '1em';
warnTooltip.style.fontWeight = 'bold';
warnTooltip.style.pointerEvents = 'none';
warnTooltip.style.zIndex = '4000';
warnTooltip.style.boxShadow = '0 2px 8px #0005';
warnTooltip.style.display = 'none';
warnTooltip.textContent = 'No flight path has been created'; // Default warning
let warning_message = 'No flight path has been created';
document.body.appendChild(warnTooltip);

function showWarnTooltip(e, message) {
    warnTooltip.textContent = message || 'No flight path has been created';
    warnTooltip.style.display = 'block';
    positionWarnTooltip(e);
}
function hideWarnTooltip() {
    warnTooltip.style.display = 'none';
}
function positionWarnTooltip(e) {
    let x = (e.touches ? e.touches[0].clientX : e.clientX) + 18;
    let y = (e.touches ? e.touches[0].clientY : e.clientY) - warnTooltip.offsetHeight - 12;
    warnTooltip.style.left = x + 'px';
    warnTooltip.style.top = y + 'px';
}

knob.addEventListener('mouseenter', (e) => {
    if (knob_status == 1) {
    arrow.style.opacity = '0';
    if (knob.classList.contains('warn')) {
        showWarnTooltip(e, warning_message);
    }
    } else {
    arrow.style.opacity = '1';
    }
    set_knob_appearance();
});
knob.addEventListener('mousemove', (e) => {
    if (knob.classList.contains('warn') && warnTooltip.style.display === 'block') {
    positionWarnTooltip(e);
    }
});
knob.addEventListener('mouseleave', () => {
    if (knob.classList.contains('active')) {
    knob.querySelector('.knob-text').textContent = 'Active';
    }
    arrow.style.opacity = '0';
    hideWarnTooltip();
});

window.socket.on('flight_status', data => {
    if (data) {
    if (data.status === 'idle') {
        console.log('Flight idle received');
        knob_status = 1; // Set to idle state
        warning_message = 'Please save a new flight path';
        reset_knob();
        set_knob_appearance();
    } else if (data.status === 'param_receive') {
        console.log('Flight parameters received by server');
        knob_status = 0; // Set to safe state
        reset_knob();
    } else if (data.status == 'flight_armed') {
        knob.querySelector('.knob-text').textContent = 'Takeoff';
    } else if (data.status == 'flight_engaged') {
        knob.querySelector('.knob-text').textContent = 'Scan';
    } else if (data.status == 'flight_landing') {
        knob.querySelector('.knob-text').textContent = 'Landing';
    } else if (data.status == 'flight_error') {
        knob.querySelector('.knob-text').textContent = 'Error';
    } else if (data.status == 'params_changed') {
        console.log('Flight warning received');
        knob_status = 1; // Set to idle state
        warning_message = 'Please save your flight path.';
        // reset_knob();
        set_knob_appearance();
    }
    }
});

function reset_knob() {
    knob.querySelector('.knob-text').textContent = 'Start';
    knob.classList.remove('active');
    knob.classList.remove('warn');
    // Right arrow SVG
    arrow.innerHTML = `
    <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
        <path d="M5 12h14M13 6l6 6-6 6" stroke="white" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
    </svg>
    `;
    arrow.style.left = '';
    arrow.style.right = '12px';
    startCondition = false;
    setKnobPosition(minLeft);
}

// Initialize
setKnobState();
setKnobPosition(minLeft);

// If you want to expose the startCondition to other scripts, you can do so here.
// window.getFlightStartCondition = () => startCondition;


const settingsBtn = document.getElementById('settingsBtn');
// const unitToggle = document.getElementById('unitToggle');
const settingsDialog = document.getElementById('settings-dialog');
let closing = false;

function open_settings() {
    settingsDialog.style.display = 'flex';
    // Force reflow to enable transition
    void settingsDialog.offsetWidth;
    settingsDialog.classList.remove('inactive');
    settingsDialog.classList.add('active');
}

function close_settings() {
    if (!settingsDialog.classList.contains('active')) return;
    closing = true;
    settingsDialog.classList.remove('active');
    settingsDialog.classList.add('inactive');
    setTimeout(() => {
    settingsDialog.style.display = 'none';
    settingsDialog.classList.remove('inactive');
    closing = false;
    }, 350);
}

settingsBtn.onclick = () => {
    open_settings();
};

settingsDialog.onclick = (e) => {
    if (e.target === settingsDialog && !closing) {
    close_settings();
    }
};
window.addEventListener('keydown', (e) => {
    if (e.key === "Escape" && settingsDialog.classList.contains('active') && !closing) {
    close_settings();
    }
});

const unitToggle = document.getElementById('unitToggle');
if (window.currentUnits === 'feet') {
    unitToggle.checked = true;
    unitToggle.setAttribute('aria-checked', 'true');
    document.getElementById('unitLabel').textContent = 'Feet';
}
const unitLabel = document.getElementById('unitLabel');
unitToggle.addEventListener('change', function() {
    if (unitToggle.checked) {
        unitLabel.textContent = 'Feet';
        unitToggle.setAttribute('aria-checked', 'true');
        // notify('Units set to Feet', 'good');
        window.socket.emit('set_units', { units: 'feet' });
    } else {
        unitLabel.textContent = 'Meters';
        unitToggle.setAttribute('aria-checked', 'false');
        // notify('Units set to Meters', 'good');
        window.socket.emit('set_units', { units: 'meters' });
    }
});
/**
 * Room Presence Sensor - Web Interface
 * Real-time status display via WebSocket
 * Per-link sensitivity control
 */

// State
let ws = null;
let isCalibrating = false;

// DOM Elements
const statusPanel = document.querySelector('.status-panel');
const statusText = document.getElementById('statusText');
const statusDetail = document.getElementById('statusDetail');
const connStatus = document.getElementById('connStatus');
const btnCalibrate = document.getElementById('btnCalibrate');
const calibCountdown = document.getElementById('calibCountdown');
const wanderThreshold = document.getElementById('wanderThreshold');
const jitterThreshold = document.getElementById('jitterThreshold');

// Initialize
document.addEventListener('DOMContentLoaded', () => {
    initWebSocket();
    initSliderListeners();
});

// Setup slider value display listeners
function initSliderListeners() {
    for (let i = 0; i < 3; i++) {
        const wSlider = document.getElementById(`link${i}WSens`);
        const jSlider = document.getElementById(`link${i}JSens`);
        const wVal = document.getElementById(`link${i}WVal`);
        const jVal = document.getElementById(`link${i}JVal`);
        
        if (wSlider && wVal) {
            wSlider.addEventListener('input', () => {
                wVal.textContent = parseFloat(wSlider.value).toFixed(2);
            });
        }
        if (jSlider && jVal) {
            jSlider.addEventListener('input', () => {
                jVal.textContent = parseFloat(jSlider.value).toFixed(2);
            });
        }
    }
}

// WebSocket Connection
function initWebSocket() {
    const wsUrl = `ws://${window.location.host}/ws`;
    
    ws = new WebSocket(wsUrl);
    
    ws.onopen = () => {
        console.log('WebSocket connected');
        connStatus.textContent = 'Connected';
        connStatus.className = 'connection-status connected';
    };
    
    ws.onclose = () => {
        console.log('WebSocket disconnected');
        connStatus.textContent = 'Disconnected';
        connStatus.className = 'connection-status disconnected';
        
        // Reconnect after 2 seconds
        setTimeout(initWebSocket, 2000);
    };
    
    ws.onerror = (error) => {
        console.error('WebSocket error:', error);
    };
    
    ws.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            updateUI(data);
        } catch (e) {
            console.error('Failed to parse message:', e);
        }
    };
}

// Update UI with sensor data
function updateUI(data) {
    // Update main status
    updateMainStatus(data.room, data.moving, data.calibrating, data.calib_remaining);
    
    // Update link cards (including per-link sensitivity display)
    for (let i = 0; i < 3; i++) {
        updateLinkCard(i, data.links[i]);
    }
    
    // Update calibration status
    isCalibrating = data.calibrating;
    updateCalibrationUI(data.calib_remaining);
    
    // Update threshold display
    if (wanderThreshold && data.wander_th !== undefined) {
        wanderThreshold.textContent = data.wander_th.toFixed(6);
    }
    if (jitterThreshold && data.jitter_th !== undefined) {
        jitterThreshold.textContent = data.jitter_th.toFixed(6);
    }
}

function updateMainStatus(room, moving, calibrating, calibRemaining) {
    statusPanel.classList.remove('empty', 'present', 'moving', 'calibrating');
    
    if (calibrating) {
        statusPanel.classList.add('calibrating');
        if (calibRemaining > 0) {
            statusText.textContent = `Calibrating... ${calibRemaining}s`;
        } else {
            statusText.textContent = 'Calibrating...';
        }
        statusDetail.textContent = 'Keep room empty';
    } else if (room && moving) {
        statusPanel.classList.add('moving');
        statusText.textContent = 'Motion Detected';
        statusDetail.textContent = 'Someone is moving';
    } else if (room) {
        statusPanel.classList.add('present');
        statusText.textContent = 'Presence Detected';
        statusDetail.textContent = 'Someone is stationary';
    } else {
        statusPanel.classList.add('empty');
        statusText.textContent = 'Room Empty';
        statusDetail.textContent = 'No presence detected';
    }
}

function updateLinkCard(index, link) {
    const card = document.getElementById(`link${index}`);
    const status = document.getElementById(`link${index}Status`);
    const wander = document.getElementById(`link${index}Wander`);
    const jitter = document.getElementById(`link${index}Jitter`);
    const bar = document.getElementById(`link${index}Bar`);
    
    // Per-link sensitivity current display (read-only, only for link 0)
    const wCurrent = document.getElementById(`link${index}WCurrent`);
    const jCurrent = document.getElementById(`link${index}JCurrent`);
    
    if (link.active) {
        card.classList.add('active');
        card.classList.remove('inactive');
        
        if (link.move) {
            status.textContent = 'Motion';
            status.className = 'link-status detecting motion';
        } else if (link.room) {
            status.textContent = 'Presence';
            status.className = 'link-status detecting';
        } else {
            status.textContent = 'Clear';
            status.className = 'link-status idle';
        }
        
        wander.textContent = link.wander.toFixed(4);
        jitter.textContent = link.jitter.toFixed(6);
        
        // Update current sensitivity display (from server, only for link 0)
        if (wCurrent && link.w_sens !== undefined) {
            wCurrent.textContent = link.w_sens.toFixed(2);
        }
        if (jCurrent && link.j_sens !== undefined) {
            jCurrent.textContent = link.j_sens.toFixed(2);
        }
        
        // Bar represents signal activity
        const barWidth = Math.min(100, link.wander * 1000);
        bar.style.width = `${barWidth}%`;
    } else {
        card.classList.remove('active');
        card.classList.add('inactive');
        status.textContent = 'Offline';
        status.className = 'link-status offline';
        wander.textContent = '--';
        jitter.textContent = '--';
        bar.style.width = '0%';
        
        if (wCurrent) wCurrent.textContent = '--';
        if (jCurrent) jCurrent.textContent = '--';
    }
}

// Per-link sensitivity apply
async function applyLinkSensitivity(linkIndex) {
    const wSlider = document.getElementById(`link${linkIndex}WSens`);
    const jSlider = document.getElementById(`link${linkIndex}JSens`);
    const btn = document.querySelector(`#link${linkIndex} .btn-sm`);
    
    if (!wSlider || !jSlider) return;
    
    const wander = parseFloat(wSlider.value);
    const jitter = parseFloat(jSlider.value);
    
    try {
        const response = await fetch('/api/sensitivity', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                link: linkIndex,
                wander_sens: wander,
                jitter_sens: jitter
            })
        });
        const result = await response.json();
        console.log(`Link ${linkIndex} sensitivity updated:`, result);
        
        // Visual feedback
        if (btn) {
            btn.textContent = '✓ OK';
            btn.classList.add('success');
            setTimeout(() => {
                btn.textContent = 'Apply';
                btn.classList.remove('success');
            }, 1500);
        }
    } catch (e) {
        console.error(`Failed to update link ${linkIndex} sensitivity:`, e);
        if (btn) {
            btn.textContent = '✗ Fail';
            setTimeout(() => {
                btn.textContent = 'Apply';
            }, 1500);
        }
    }
}

// Calibration
function toggleCalibration() {
    if (isCalibrating) {
        stopCalibration();
    } else {
        startCalibration();
    }
}

async function startCalibration() {
    try {
        const response = await fetch('/api/calibrate', {
            method: 'POST',
            body: 'start'
        });
        const result = await response.json();
        console.log('Calibration started:', result);
    } catch (e) {
        console.error('Failed to start calibration:', e);
    }
}

async function stopCalibration() {
    try {
        const response = await fetch('/api/calibrate', {
            method: 'POST',
            body: 'stop'
        });
        const result = await response.json();
        console.log('Calibration complete:', result);
    } catch (e) {
        console.error('Failed to stop calibration:', e);
    }
}

function updateCalibrationUI(calibRemaining) {
    if (isCalibrating) {
        btnCalibrate.textContent = 'Stop Calibration';
        btnCalibrate.classList.add('active');
        if (calibCountdown && calibRemaining > 0) {
            calibCountdown.textContent = `${calibRemaining}s remaining`;
        }
    } else {
        btnCalibrate.textContent = 'Start Calibration (30s)';
        btnCalibrate.classList.remove('active');
        if (calibCountdown) {
            calibCountdown.textContent = '';
        }
    }
}

// Fallback polling if WebSocket not supported
if (!window.WebSocket) {
    setInterval(() => {
        fetch('/api/status')
            .then(r => r.json())
            .then(data => updateUI(data))
            .catch(e => console.error('Poll failed:', e));
    }, 1000);
}

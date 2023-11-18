// JavaScript for handling the state machines and timers
function updateStateMachine(state_len, state, prefix) {
    for (let i = 0; i <= state_len; i++) {
        const stateCircle = document.getElementById(`${prefix}-state-${i}`);
        stateCircle.classList.remove('active');
    }
    const activeState = document.getElementById(`${prefix}-state-${state}`);
    if (activeState) {
        activeState.classList.add('active');
    }
}

function updateAmigaStateMachine(state) {
    console.log(state);
    const plants = document.querySelectorAll('#plant-bed .plant');
    plants.forEach(plant => plant.classList.remove('blink-shadow'));
    if (state >= 0 && state < plants.length) {
        plants[state].classList.add('blink-shadow');
    }
}

// Timer functionality
let totalSeconds = 0;
let interval = null;

function startTimer() {
    if (interval === null) {
        interval = setInterval(() => {
            totalSeconds++;
            updateTimerDisplay();
        }, 1000);
    }
}

function stopTimer() {
    if (interval !== null) {
        clearInterval(interval);
        interval = null;
    }
}

function updateTimerDisplay() {
    const timerElement = document.getElementById('timer-display');
    timerElement.textContent = `Total Time: ${formatTime(totalSeconds)}`;
}

function formatTime(seconds) {
    const minutes = Math.floor(seconds / 60);
    const hours = Math.floor(minutes / 60);
    const remainingMinutes = minutes % 60;
    const remainingSeconds = seconds % 60;
    return `${padZero(hours)}:${padZero(remainingMinutes)}:${padZero(remainingSeconds)}`;
}

function padZero(num) {
    return num.toString().padStart(2, '0');
}

let harvestTimes = [];

function recordHarvestTime() {
    stopTimer();
    harvestTimes.push(totalSeconds);
    appendHarvestTimeToList(totalSeconds);
    calculateAndDisplayAverage();
    saveData(); // Save data after recording a new harvest time
    totalSeconds = 0; // Reset the timer for the next harvest
    startTimer();
}

function appendHarvestTimeToList(seconds) {
    const list = document.getElementById('harvest-times-list');
    const listItem = document.createElement('li');
    listItem.textContent = `Harvest Time: ${formatTime(seconds)}`;
    list.appendChild(listItem);
}


function calculateAndDisplayAverage() {
    const average = harvestTimes.reduce((a, b) => a + b, 0) / harvestTimes.length;
    const averageElement = document.getElementById('average-time');
    averageElement.textContent = `Average Time: ${formatTime(Math.round(average))}`;
}

function saveData() {
    localStorage.setItem('harvestTimes', JSON.stringify(harvestTimes));
    localStorage.setItem('totalSeconds', totalSeconds.toString());
}

function loadData() {
    const savedTimes = localStorage.getItem('harvestTimes');
    const savedTotalSeconds = localStorage.getItem('totalSeconds');
    if (savedTimes) {
        harvestTimes = JSON.parse(savedTimes);
        harvestTimes.forEach(seconds => appendHarvestTimeToList(seconds));
        calculateAndDisplayAverage();
    }
    if (savedTotalSeconds) {
        totalSeconds = parseInt(savedTotalSeconds);
        updateTimerDisplay();
    }
}

// Save data when the page is about to be unloaded
window.addEventListener('beforeunload', saveData);

// Initialization and Socket Events
document.addEventListener('DOMContentLoaded', () => {
    const socket = io.connect(location.protocol + '//' + document.domain + ':' + location.port);

    socket.on('system_state_update', function (data) {
        updateStateMachine(10, data.state, 'system');
        // Example: Start or stop the timer based on the state
        if (data.state === 3) { // Assuming state 3 is when harvesting starts
            startTimer();
        } else if (data.state === 8) { // Assuming state 8 is when harvesting ends
            recordHarvestTime();
        }
    });

    socket.on('amiga_state_update', function (data) {
        if (data.state > 0) {
            data.state = (data.state - 22) / 2;
        } else {
            data.state = -1;
        }
        updateAmigaStateMachine(data.state);
    });
    window.onload = loadData;

});

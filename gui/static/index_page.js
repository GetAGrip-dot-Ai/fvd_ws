const systemStatesManual = [
    { title: '[0] IDLE/Amiga Teleop', value: '[0] idle' },
    { title: '[1] xArm Teleop', value: '[1] teleop' },
    { title: '[2] Manual: Move to Init', value: '[2] visual servo' },
    { title: '[10] Error', value: '[10] Error' }
];

const systemStatesAutonomous = [
    { title: '[3] Move to Init', value: '[3] return to init' },
    { title: '[4] Multiframe', value: '[4] move to init pose' },
    { title: '[5] Move to Pregrasp', value: '[5] multiframe' },
    { title: '[6] Move to POI', value: '[6] move to pregrasp' },
    { title: '[7] Harvest', value: '[7] move to poi' },
    { title: '[8] Move to basket', value: '[8] harvest pepper' },
    { title: '[9] Release & Reset', value: '[9] basket drop' },
];
const amigaStates = [
    { title: 'Manual', value: 'Not Moving' },
    { title: 'Manual', value: 'Manually Moving' },
    { title: 'Autonomous', value: 'Autonomously Moving' },
    { title: 'Manual', value: 'IDK' },
];
// JavaScript for handling the state machines and timers
function createStateCircles(states, containerId, prefix) {
    const container = document.getElementById(containerId);
    container.classList.add('state-machine-container'); // Add a container class

    states.forEach((state, index) => {
        const stateRectangle = document.createElement('div');
        stateRectangle.className = 'state-circle';
        stateRectangle.id = `${prefix}-state-${index}`;

        const stateTitle = document.createElement('div');
        stateTitle.className = 'state-title';
        stateTitle.textContent = state.title;

        stateRectangle.appendChild(stateTitle);

        container.appendChild(stateRectangle);

        if (index < states.length - 1) {
            const arrow = document.createElement('div');
            arrow.className = 'arrow';
            container.appendChild(arrow);
        }
    });
}

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

// Timer functionality

let timers = {
    timer1: { element: null, interval: null, seconds: 0 },
};
function formatTime(seconds) {
    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    const remainingSeconds = seconds % 60;
    return `${padZero(hours)}:${padZero(minutes)}:${padZero(remainingSeconds)}`;
}

function padZero(num) {
    return num.toString().padStart(2, '0');
}


function updateTimerDisplay(timerId) {
    const timer = timers[timerId];
    timer.element.textContent = `Timer ${timerId}: ${formatTime(timer.seconds)}`;
}


function startTimer(timerId) {
    stopAllTimers(); // Optional: stop all other timers when one starts
    const timer = timers[timerId];
    timer.interval = setInterval(() => {
        timer.seconds++;
        updateTimerDisplay(timerId);
    }, 1000);
}

function stopTimer(timerId) {
    const timer = timers[timerId];
    clearInterval(timer.interval);
    timer.interval = null;
}

function stopAllTimers() {
    Object.keys(timers).forEach(stopTimer);
}
function addNewTimer(timerId, pepperId) {
    // Check if the timer already exists
    if (!timers[timerId]) {
        // Create a new timer object
        timers[timerId] = { element: null, interval: null, seconds: 0 };

        // Create the timer display element
        const timerElement = document.createElement('div');
        timerElement.id = `timer-${timerId}`;

        // Add the timer display to the page
        document.getElementById('timers').appendChild(timerElement);

        // Apply CSS classes
        timerElement.className = 'timer';
        
        const titleElement = document.createElement('div');
        titleElement.className = 'timer-title';
        titleElement.textContent = `Timer ${pepperId}`;
        timerElement.appendChild(titleElement);

        const timeElement = document.createElement('div');
        timeElement.id = `time-${timerId}`;
        timeElement.textContent = '00:00:00';
        timerElement.appendChild(timeElement);

        // Update the timer reference to the time display
        timers[timerId].element = timeElement;
    }
}

// Initialization

document.addEventListener('DOMContentLoaded', () => {
    // createStateCircles(systemStatesManual, 'system-state-manual-machine', 'system', 'state-machine-container'); // Add 'state-machine-container' class
    // createStateCircles(systemStatesAutonomous, 'system-state-autonomous-machine', 'system', 'state-machine-container'); // Add 'state-machine-container' class
    // createStateCircles(amigaStates, 'amiga-state-machine', 'amiga', 'state-machine-container'); // Add 'state-machine-container' class

    const socket = io.connect(location.protocol + '//' + document.domain + ':' + location.port);

    // Listen for state update events
    socket.on('system_state_update', function (data) {
        updateStateMachine(10, data.state, 'system');
    });


    socket.on('timer_update', function (data) {
        // Assuming data.elapsed_time contains the elapsed time in seconds
        const elapsedSeconds = Math.round(data.elapsed_time);
        const pepper_id = data.pepper_id;

        const timerKey = 'timer' + pepper_id;
        console.log('Received timer_update:', pepper_id, " | ", data.elapsed_time);

        // Add a new timer if it doesn't exist
        addNewTimer(timerKey, pepper_id);

        // Update the specific timer here
        // For example, updating timer1. Adjust this based on your logic
        timers[timerKey].seconds = elapsedSeconds;
        // print in console the value it tis getting
        updateTimerDisplay(timerKey);
    });


    socket.on('amiga_state_update', function (data) {
        updateStateMachine(3, data.state, 'amiga');
    });

});

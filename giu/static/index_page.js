const systemStates = [
    { title: 'Manual', value: '[0] idle' },
    { title: 'Manual', value: '[1] teleop' },
    { title: 'Manual', value: '[2] visual servo' },
    { title: 'Manual', value: '[3] return to init' },
    { title: 'Autonomous', value: '[4] move to init pose' },
    { title: 'Autonomous', value: '[5] multiframe' },
    { title: 'Autonomous', value: '[6] move to pregrasp' },
    { title: 'Autonomous', value: '[7] move to poi' },
    { title: 'Autonomous', value: '[8] harvest pepper' },
    { title: 'Autonomous', value: '[9] basket drop' },
    { title: 'Autonomous', value: '[10] Error' }
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

        const stateValue = document.createElement('div');
        stateValue.className = 'state-value';
        stateValue.textContent = state.value;

        stateRectangle.appendChild(stateTitle);
        stateRectangle.appendChild(stateValue);

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
let timers = [];

function startTimer(timerId) {
    const timerElement = document.getElementById(`timer-${timerId}`);
    let seconds = 0;
    timers[timerId] = setInterval(() => {
        seconds++;
        timerElement.textContent = `Timer ${timerId}: ${new Date(seconds * 1000).toISOString().substr(11, 8)}`;
    }, 1000);
}


function createTimer(timerId) {
    const timerButton = document.createElement('button');
    timerButton.className = 'timer-button';
    timerButton.id = `timer-${timerId}`;
    timerButton.textContent = `Timer ${timerId}: 00:00:00`;
    let timerInterval;

    timerButton.addEventListener('click', function () {
        if (timerInterval) {
            clearInterval(timerInterval);
            timerInterval = null;
        } else {
            let seconds = 0;
            timerInterval = setInterval(() => {
                seconds++;
                timerButton.textContent = `Timer ${timerId}: ${formatTime(seconds)}`;
            }, 1000);
        }
    });

    const timersContainer = document.getElementById('timers');
    timersContainer.appendChild(timerButton);
}

function formatTime(seconds) {
    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    const remainingSeconds = seconds % 60;
    return `${padZero(hours)}:${padZero(minutes)}:${padZero(remainingSeconds)}`;
}

function padZero(num) {
    return num.toString().padStart(2, '0');
}

// Initialization

document.addEventListener('DOMContentLoaded', () => {
    createStateCircles(systemStates, 'system-state-machine', 'system', 'state-machine-container'); // Add 'state-machine-container' class
    createStateCircles(amigaStates, 'amiga-state-machine', 'amiga', 'state-machine-container'); // Add 'state-machine-container' class
    for (let i = 0; i <= 6; i++) {
        createTimer(i);
    }

    const socket = io.connect(location.protocol + '//' + document.domain + ':' + location.port);

    // Listen for state update events
    socket.on('system_state_update', function (data) {
        updateStateMachine(10, data.state, 'system');
    });

    socket.on('amiga_state_update', function (data) {
        console.log('Received amiga_state_update:', data.state);
        updateStateMachine(3, data.state, 'amiga');
    });

});

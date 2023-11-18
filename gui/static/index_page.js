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
function updateAmigaStateMachine(state) {
    console.log(state);
    const plants = document.querySelectorAll('#plant-bed .plant');
    plants.forEach(plant => plant.classList.remove('blink-shadow')); // Remove blinking from all plants

    if (state >= 0 && state < plants.length) {
        // Apply blinking to the specific plant
        plants[state].classList.add('blink-shadow');
    }
}


// Timer functionality
let timers = {
    timer1: { element: null, interval: null, seconds: 0 },
};
function formatTime(seconds) {
    const minutes = Math.floor((seconds % 3600) / 60);
    const remainingSeconds = seconds % 60;
    return `${padZero(minutes)}:${padZero(remainingSeconds)}`;
}

function padZero(num) {
    return num.toString().padStart(2, '0');
}


function updateTimerDisplay(timerId) {
    const timer = timers[timerId];
    timer.element.textContent = `Pepper Harvest Time: ${formatTime(timer.seconds)}`;
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
        // console.log('Received timer_update:', pepper_id, " | ", data.elapsed_time);

        // Add a new timer if it doesn't exist
        addNewTimer(timerKey, pepper_id);

        // Update the specific timer here
        // For example, updating timer1. Adjust this based on your logic
        timers[timerKey].seconds = elapsedSeconds;
        // print in console the value it tis getting
        updateTimerDisplay(timerKey);
    });

    socket.on('amiga_state_update', function (data) {
        if (data.state>0){
            data.state = (data.state - 22) / 2;
        }
        else{
            data.state = -1;
        }
        updateAmigaStateMachine(data.state);
        // console.log('Received amiga_state_update:', data.state);
    });

    // const plants = document.querySelectorAll('.plant');
    // if (plants.length > 0) {
    //     // Applying the blinking effect to the first plant
    //     plants[0].style.animation = "blink-shadow 1s infinite";
    // }

});

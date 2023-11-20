#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
import threading
from flask import Flask, render_template
from flask_socketio import SocketIO
import time

app = Flask(__name__)
socketio = SocketIO(app)

# Using lock for thread-safe operations on shared global variables
lock = threading.Lock()

# Declaring global variables
global system_state, amiga_state
system_state = -1
amiga_state = -1


def system_state_callback(msg):
    global system_state
    with lock:  # Ensuring thread safety
        system_state = msg.data
        if system_state == 10:
            system_state = 9

    # Emitting the updated state to the socket
    socketio.emit('system_state_update', {'state': system_state})


def amiga_state_callback(msg):
    global amiga_state
    with lock:  # Ensuring thread safety
        amiga_state = msg.data

    # Emitting the updated state to the socket
    socketio.emit('amiga_state_update', {'state': amiga_state})


def ros_thread():
    rospy.Subscriber('/state', Int16, system_state_callback)
    rospy.Subscriber('/amiga_aligned', Int16, amiga_state_callback)

    # Handling ROS spin in a try-except block
    try:
        rospy.spin()
    except Exception as e:
        print("ROS thread error:", e)
        # Additional error handling logic can be added here


@app.route('/')
def index():
    return render_template('index.html')


if __name__ == '__main__':
    rospy.init_node('peter_gui', anonymous=True, disable_signals=True)

    # Starting the ROS subscriber thread
    t = threading.Thread(target=ros_thread)
    t.daemon = True
    t.start()

    # Starting the Flask app
    try:
        socketio.run(app, host='0.0.0.0', port=5000, debug=False)
    except Exception as e:
        print("Flask server error:", e)
        # Additional error handling logic can be added here

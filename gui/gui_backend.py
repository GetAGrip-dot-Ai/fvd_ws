#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int16, Bool

from cv_bridge import CvBridge
import cv2
import numpy as np

import threading
import base64

from flask import Flask, render_template, Response, request, jsonify
from flask_socketio import SocketIO, emit
import time
from collections import defaultdict


app = Flask(__name__)

socketio = SocketIO(app)
lock = threading.Lock()

system_state = -1
amiga_state = -1

harvest_times = defaultdict()
pepper_id = 1


timer_start = time.time()
elapsed_time = 0

def system_state_callback(msg):
    global system_state, timer_start, elapsed_time, pepper_id

    previous_state = system_state
    system_state = msg.data

    # Start timer when state changes to 3
    if system_state == 3 and previous_state != 3:
        pepper_id += 1
        timer_start = time.time()

    # Stop timer and calculate elapsed time when state changes to 8
    elif system_state == 8 and previous_state != 8 and timer_start is not None:
        harvest_times[pepper_id] = time.time() - timer_start
    if system_state < 3:
        pass
    else:
        elapsed_time = time.time() - timer_start
    
        # timer_start = None
    socketio.emit('timer_update', {'pepper_id': pepper_id, 'elapsed_time': elapsed_time})
    # print("Sending timer update", elapsed_time)

    socketio.emit('system_state_update', {'state': system_state})



def amiga_state_callback(msg):
    global amiga_state
    amiga_state = msg.data
    socketio.emit('amiga_state_update', {'state': amiga_state})


def ros_thread():
    rospy.Subscriber('/state', Int16, system_state_callback)
    rospy.Subscriber('/amiga_aligned', Int16, amiga_state_callback)
    rospy.spin()


@app.route('/')
def index():
    return render_template('index.html')




if __name__ == '__main__':
    rospy.init_node('peter_gui', anonymous=True, disable_signals=True)
    t = threading.Thread(target=ros_thread)
    t.daemon = True
    t.start()
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)

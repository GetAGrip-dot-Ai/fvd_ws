#!/usr/bin/env python3

# app.py (continued)
from flask import Flask, render_template, request
import rospy  # Import ROS libraries
from std_msgs.msg import String  # Replace with appropriate ROS message type

app = Flask(__name__)

# Initialize ROS Node
rospy.init_node('web_image_point_selector')

# Define ROS Publisher
# Replace with your ROS topic and message type
ros_publisher = rospy.Publisher('/your_ros_topic', String, queue_size=10)


@app.route('/')
def index():
    image_url = "./static/pink_jelly.png"  # Replace with the path to your image
    return render_template('index.html', image_url=image_url)


@app.route('/send_to_ros', methods=['POST'])
def send_to_ros():
    if request.method == 'POST':
        data = request.json
        x = data['x']
        y = data['y']

        # Publish the data to the ROS topic
        ros_publisher.publish(f'({x}, {y})')

        return 'Data sent to ROS'


if __name__ == '__main__':
    app.run(debug=True)

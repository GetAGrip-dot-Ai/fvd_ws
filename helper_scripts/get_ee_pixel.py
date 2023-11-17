#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image as RosImage  # Renamed for clarity
from cv_bridge import CvBridge
import tkinter as tk
from PIL import Image, ImageTk
from geometry_msgs.msg import Point

# Global variables
bridge = CvBridge()
cv_image = None
click_position = None  # Store the click position
image_counter = 10  # Initialize counter to a value >= 10 to not draw initially
custom_poi_publisher = rospy.Publisher("/user_selected_poi", Point, queue_size=10)

def on_click(event):
    global click_position, image_counter
    click_position = (event.x, event.y)
    image_counter = 0  # Reset counter on new click
    print(f"Clicked at: x={event.x}, y={event.y}")
    selected_point = Point(click_position[0], click_position[1], 0)
    custom_poi_publisher.publish(selected_point)

def image_callback(msg):
    global cv_image, image_counter
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    if image_counter < 10:
        image_counter += 1  # Increment the counter for each new image

def refresh_image():
    if cv_image is not None:
        # If there was a click and the counter is less than 10, draw the red point
        if click_position and image_counter < 10:
            cv2.circle(cv_image, click_position, 5, (0, 0, 255), -1)

        # Convert the OpenCV image to PIL format
        pil_image = Image.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        tk_image = ImageTk.PhotoImage(image=pil_image)

        # Update the image of the label
        label.config(image=tk_image)
        label.image = tk_image  # Keep a reference!
    
    # Re-run the function after a short delay
    root.after(100, refresh_image)

def main():
    rospy.init_node("pixel_picker")
    image_topic = "/camera/color/image_raw"  # Change this to your image topic name
    rospy.Subscriber(image_topic, RosImage, image_callback)

    global root, label

    # Setup the main window
    root = tk.Tk()
    root.title("Choose your POI")

    # Initially, we display a placeholder image or a blank label
    label = tk.Label(root)
    label.pack()

    # Bind click event
    label.bind("<Button-1>", on_click)

    # Refresh the image on the label
    refresh_image()

    # Start the GUI event loop
    root.mainloop()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

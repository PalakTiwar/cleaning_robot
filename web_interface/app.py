import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, render_template, request
import threading
import time
import sys

# --- ROS 2 Node setup ---
class RoomPublisherNode(Node):
    def __init__(self):
        super().__init__('room_cleaning_publisher')
        self.publisher_ = self.create_publisher(String, 'room_to_clean', 10)
        self.get_logger().info('ROS 2 Room Cleaning Publisher Node initialized.')

    def publish_room_command(self, room_name):
        msg = String()
        msg.data = room_name
        self.publisher_.publish(msg)
        self.get_logger().info(f'ROS 2 Published: "{room_name}" to /room_to_clean')

# Global variable for the ROS 2 node instance
ros_node_instance = None

# Function to run the ROS 2 spin in a separate thread
def ros_spin_thread():
    global ros_node_instance
    rclpy.init(args=None)
    ros_node_instance = RoomPublisherNode()
    rclpy.spin(ros_node_instance)
    ros_node_instance.destroy_node()
    rclpy.shutdown()
    print("ROS 2 thread shut down.")


# --- Flask App setup ---
app = Flask(__name__)

# Route for the main UI page (home.html)
@app.route('/')
def home_page():
    # Renders the 'home.html' file from the 'templates' directory
    return render_template('home.html')

# Route for handling cleaning commands
@app.route('/start-cleaning')
def start_cleaning():
    room = request.args.get('room')
    if room:
        if ros_node_instance:
            ros_node_instance.publish_room_command(room)
            # Your existing print statement for Flask console output
            print(f"Published: {room}")
            # --- IMPORTANT CHANGE HERE ---
            # Instead of returning a string for client-side JS,
            # we now render the status.html template directly from the server.
            # This will redirect the browser to the status page.
            return render_template('status.html', room=room)
        else:
            print("Error: ROS 2 node not initialized.")
            return "ROS 2 node not ready. Please try again."
    return "No room specified."

# Route for the status page (now explicitly uncommented and callable directly)
@app.route('/status_page') # Added a different route name to avoid conflict if /status is also used via /start-cleaning
def status_page():
    # You might pass dynamic data to status.html here
    # For a direct visit to /status_page, 'room' won't be in request.args
    # You could fetch it from a session or a global variable if needed,
    # or handle cases where it's not present.
    # For now, let's assume it's primarily rendered after a cleaning command.
    return render_template('status.html', room="Unknown Room") # Provide a default or handle missing 'room'

# --- Main App Execution ---
if __name__ == '__main__':
    # Start the ROS 2 spin in a separate daemon thread
    ros_thread = threading.Thread(target=ros_spin_thread)
    ros_thread.daemon = True
    ros_thread.start()

    # Give the ROS 2 node a moment to initialize
    time.sleep(1)

    try:
        app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)
    except KeyboardInterrupt:
        print("Flask app shutting down.")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        time.sleep(0.5)
        sys.exit(0)

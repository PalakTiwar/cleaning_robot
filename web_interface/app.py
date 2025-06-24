from flask import Flask, render_template, request
import rclpy
from std_msgs.msg import String
import threading

app = Flask(__name__)
rclpy.init()
node = rclpy.create_node('web_command_node')
publisher = node.create_publisher(String, 'clean_room_cmd', 10)
cleaning_status = {
'room': None,
'progress': 0,
'start_time': None,
'end_time': None
}
def ros_spin():
    rclpy.spin(node)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/clean', methods=['POST'])
def clean():
    room = request.form.get('room')
    # Publish room name to ROS 2 topic
    msg = String()
    msg.data = room
    publisher.publish(msg)
    print(f"[ROS] Published cleaning command for: {room}")

    # Initialize cleaning status
    cleaning_status['room'] = room
    cleaning_status['progress'] = 0
    cleaning_status['start_time'] = time.time()
    cleaning_status['end_time'] = None
    # Start fake cleaning simulation in background
    threading.Thread(target=simulate_cleaning, daemon=True).start()
    # Redirect to cleaning status page
    
    return redirect(url_for('status'))
    
@app.route('/status')
def status():
    room = cleaning_status['room']
    progress = cleaning_status['progress']
    time_taken = None
    if cleaning_status['end_time']:
        time_taken = round(cleaning_status['end_time'] - cleaning_status['start_time'], 2)

    return render_template('status.html',
                       room=room,
                       progress=progress,
                       time_taken=time_taken)
                       
def simulate_cleaning():
    while cleaning_status['progress'] < 100:
    time.sleep(0.5) # simulate cleaning time
    cleaning_status['progress'] += 10
    cleaning_status['end_time'] = time.time()      
if __name__ == '__main__':
    threading.Thread(target=ros_spin, daemon=True).start()
    app.run(host='127.0.0.1', port=5000)


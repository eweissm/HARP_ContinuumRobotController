import sys
import time
import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from NatNetClient import NatNetClient
import DataDescriptions
import MoCapData

# Store latest rigid body positions
rigid_body_positions = {}

def receive_new_frame(data_dict):
    pass  # not needed for visualization

def receive_rigid_body_frame(new_id, position, rotation):
    # Update global dictionary with latest rigid body positions
    rigid_body_positions[new_id] = position

def update_plot(frame_num, scat, ax):
    if not rigid_body_positions:
        return scat,

    xs, ys, zs = [], [], []
    labels = []

    for rb_id, pos in rigid_body_positions.items():
        xs.append(pos[0])
        ys.append(pos[1])
        zs.append(pos[2])
        labels.append(str(rb_id))

    scat._offsets3d = (xs, ys, zs)
    ax.clear()
    ax.set_xlim(-.05, .1)
    ax.set_ylim(0, 1)
    ax.set_zlim(-.1, 0)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Real-time Rigid Body Positions")
    ax.scatter(xs, ys, zs)

    for i, label in enumerate(labels):
        ax.text(xs[i], ys[i], zs[i], label)

    return scat,

def start_natnet_client():
    optionsDict = {
        "clientAddress": "127.0.0.1",
        "serverAddress": "127.0.0.1",
        "use_multicast": True,
        "stream_type": 'd'
    }

    client = NatNetClient()
    client.set_client_address(optionsDict["clientAddress"])
    client.set_server_address(optionsDict["serverAddress"])
    client.set_use_multicast(optionsDict["use_multicast"])

    client.new_frame_listener = receive_new_frame
    client.rigid_body_listener = receive_rigid_body_frame

    is_running = client.run(optionsDict["stream_type"])
    if not is_running:
        print("ERROR: Could not start streaming client.")
        sys.exit(1)

    time.sleep(1)
    if not client.connected():
        print("ERROR: Could not connect to server.")
        sys.exit(2)

    print("NatNet client connected successfully.")

# Run NatNet client in a background thread
# natnet_thread = threading.Thread(target=start_natnet_client, daemon=True)
# natnet_thread.start()
start_natnet_client()

# Set up 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
scat = ax.scatter([], [], [])

ani = FuncAnimation(fig, update_plot, fargs=(scat, ax), interval=100)
plt.show()

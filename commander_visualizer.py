# commander_visualizer.py

import paho.mqtt.client as mqtt
import json
import time
import uuid
import datetime
import threading  # To run MQTT loop and visualization concurrently
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

# --- Import necessary components from your project ---
# (Make sure these paths are correct relative to where you run this script)
import config  # To get broker details and config
import mqtt_utils # For base topic generation
import utils # For timestamp

# Import VDA5050 message classes (adjust paths as needed)
from protocol.vda5050_common import AgvPosition, NodePosition
from protocol.vda_2_0_0.vda5050_2_0_0_order import Order, Node, Edge
from protocol.vda_2_0_0.vda5050_2_0_0_instant_actions import InstantActions
from protocol.vda_2_0_0.vda5050_2_0_0_action import Action, ActionParameter, ActionParameterValue, BlockingType
from protocol.vda_2_0_0.vda5050_2_0_0_visualization import Visualization

# --- Global Variables ---
config_data = config.get_config()
robot_positions = {} # Dictionary to store latest positions: {serial_number: (x, y, theta)}
mqtt_client = None
robot_serial_numbers = [f"{config_data.vehicle.serial_number}{i}" for i in range(config_data.settings.robot_count)]
base_topics = {} # {serial_number: base_topic_string}

# Plotting variables
fig, ax = None, None
scatter_plots = {} # {serial_number: plot_object}
text_labels = {} # {serial_number: text_object}


# --- MQTT Functions ---

def get_robot_base_topic(serial_number):
    """Generates the VDA5050 base topic for a specific robot."""
    return mqtt_utils.generate_vda_mqtt_base_topic(
        config_data.mqtt_broker.vda_interface,
        config_data.vehicle.vda_version,
        config_data.vehicle.manufacturer,
        serial_number # Use the specific robot's serial
    )

def on_connect(client, userdata, flags, rc, properties=None):
    print(f"Commander connected with result code {rc}")
    # Subscribe to visualization topics for all robots
    for serial in robot_serial_numbers:
        base_topic = base_topics.get(serial)
        if base_topic:
            vis_topic = f"{base_topic}/visualization"
            client.subscribe(vis_topic, qos=1)
            print(f"Subscribed to {vis_topic}")

def on_message(client, userdata, msg):
    topic = msg.topic
    payload = msg.payload.decode('utf-8')
    print(f"Received message on {topic}")
    print(f"Received message on {payload}")

    # Identify the robot and topic type
    topic_parts = topic.split('/')
    if len(topic_parts) >= 5 and topic_parts[-1] == "visualization":
        serial_number = topic_parts[-2]
        if serial_number in robot_serial_numbers:
            try:
                vis_data = json.loads(payload)
                # Basic validation: Check if essential keys exist
                if "agvPosition" in vis_data and isinstance(vis_data["agvPosition"], dict):
                     pos = vis_data["agvPosition"]
                     if all(k in pos for k in ("x", "y", "theta")):
                          robot_positions[serial_number] = (pos["x"], pos["y"], pos["theta"])
                          # print(f"Updated position for {serial_number}: {robot_positions[serial_number]}")
                     else:
                          print(f"Warning: Missing x, y, or theta in agvPosition for {serial_number}")
                else:
                     print(f"Warning: Missing or invalid agvPosition in message for {serial_number}")

            except json.JSONDecodeError:
                print(f"Error decoding JSON from {topic}")
            except Exception as e:
                print(f"Error processing message from {topic}: {e}")

def setup_mqtt():
    global mqtt_client
    opts = mqtt_utils.mqtt_create_opts() # Generate a unique client ID
    mqtt_client = mqtt.Client(client_id=opts["client_id"] + "-commander", protocol=mqtt.MQTTv5)
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message

    # Generate base topics for all robots
    for serial in robot_serial_numbers:
        base_topics[serial] = get_robot_base_topic(serial)

    try:
        mqtt_client.connect(
            host=config_data.mqtt_broker.host,
            port=int(config_data.mqtt_broker.port)
        )
        mqtt_client.loop_start() # Start network loop in background thread
    except Exception as e:
        print(f"MQTT connection failed: {e}")
        mqtt_client = None # Indicate failure


# --- Command Sending Functions ---

def send_instant_action(serial_number, action_type, params={}, blocking_type=BlockingType.HARD):
    """Sends an InstantAction command to a specific robot."""
    if not mqtt_client:
        print("MQTT client not connected.")
        return

    base_topic = base_topics.get(serial_number)
    if not base_topic:
        print(f"Could not find base topic for {serial_number}")
        return

    action_params = [ActionParameter(key=k, value=ActionParameterValue(v)) for k, v in params.items()]

    action = Action(
        action_type=action_type,
        action_id=str(uuid.uuid4()), # Unique action ID
        blocking_type=blocking_type,
        action_parameters=action_params
    )

    instant_action_cmd = InstantActions(
        header_id=0, # Consider managing header IDs more robustly
        timestamp=utils.get_timestamp(),
        version=config_data.vehicle.vda_full_version,
        manufacturer=config_data.vehicle.manufacturer,
        serial_number=serial_number, # Target specific robot
        instant_actions=[action]
    )

    topic = f"{base_topic}/instantActions"
    payload = json.dumps(instant_action_cmd.to_dict())

    print(f"Sending to {topic}: {payload}")
    result = mqtt_client.publish(topic, payload=payload, qos=1)
    # result.wait_for_publish() # Optional: wait for publish confirmation
    print(f"Publish result: {result.rc}")


def send_order(serial_number, order_id, nodes_data, edges_data):
     """Sends an Order command to a specific robot.
     nodes_data: list of dicts like {'node_id': 'N1', 'x': 1.0, 'y': 2.0, 'actions': []}
     edges_data: list of dicts like {'edge_id': 'E1', 'start_node_id': 'N1', 'end_node_id': 'N2'}
     """
     if not mqtt_client:
        print("MQTT client not connected.")
        return

     base_topic = base_topics.get(serial_number)
     if not base_topic:
        print(f"Could not find base topic for {serial_number}")
        return

     nodes = []
     sequence_id_counter = 0
     for i, n_data in enumerate(nodes_data):
          sequence_id_counter += 1
          # TODO: Add parsing for actions within nodes if needed
          nodes.append(Node(
               node_id=n_data['node_id'],
               sequence_id=sequence_id_counter,
               released=True, # Assume nodes are initially released
               node_position=NodePosition(
                    x=n_data['x'],
                    y=n_data['y'],
                    map_id=config_data.settings.map_id # Use map from config
               ),
               actions=[] # Add actions here if needed
          ))

     edges = []
     for i, e_data in enumerate(edges_data):
          sequence_id_counter += 1
           # TODO: Add parsing for actions within edges if needed
          edges.append(Edge(
               edge_id=e_data['edge_id'],
               sequence_id=sequence_id_counter,
               start_node_id=e_data['start_node_id'],
               end_node_id=e_data['end_node_id'],
               released=True,
               actions=[] # Add actions here if needed
          ))

     order_cmd = Order(
          header_id=0, # Manage header IDs
          timestamp=utils.get_timestamp(),
          version=config_data.vehicle.vda_full_version,
          manufacturer=config_data.vehicle.manufacturer,
          serial_number=serial_number,
          order_id=order_id,
          order_update_id=0, # Manage update IDs
          nodes=nodes,
          edges=edges
     )

     topic = f"{base_topic}/order"
     payload = json.dumps(order_cmd.to_dict())

     print(f"Sending to {topic}: {payload}")
     result = mqtt_client.publish(topic, payload=payload, qos=1)
     print(f"Publish result: {result.rc}")


# --- Visualization Functions ---

def init_plot():
    """Initializes the Matplotlib plot axes and elements."""
    # --- >> REMOVE: fig, ax = plt.subplots() << ---  (This line is moved)
    ax.set_xlabel("X Coordinate")
    ax.set_ylabel("Y Coordinate")
    ax.set_title("Robot Positions")
    ax.grid(True)
    # Set initial plot limits (adjust as needed)
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_aspect('equal', adjustable='box') # Make axes scale equally

    # Create plot objects for each robot
    initial_artists = []
    for serial in robot_serial_numbers:
        # Initial dummy plot points, updated later
        scatter, = ax.plot([], [], marker='o', linestyle='', markersize=8, label=serial)
        scatter_plots[serial] = scatter
        # Add text label near the plot point
        text = ax.text(0, 0, serial[-2:], fontsize=8, ha='center', va='bottom') # Show last 2 chars
        text_labels[serial] = text
        initial_artists.extend([scatter, text]) # Add elements to return for blitting

    ax.legend()
    return initial_artists # Return iterable of artists created

def update_plot(frame):
    """Called periodically by FuncAnimation to update the plot."""
    updates = []
    for serial, plot_obj in scatter_plots.items():
        if serial in robot_positions:
            x, y, theta = robot_positions[serial]
            plot_obj.set_data([x], [y]) # Update scatter plot position
            text_labels[serial].set_position((x, y + 0.2)) # Update text label position slightly above
            updates.append(plot_obj)
            updates.append(text_labels[serial])
        else:
            # Keep robots without position data hidden initially
             plot_obj.set_data([], [])
             text_labels[serial].set_position((0, -1000)) # Move text off-screen
    return updates


# --- Main Execution ---

if __name__ == "__main__":
    print("Starting Commander and Visualizer...")
    setup_mqtt()

    if not mqtt_client:
        print("Exiting due to MQTT connection failure.")
        exit()

    print("MQTT Client Setup Complete.")

    # --- Example Commands (Uncomment to send) ---
    time.sleep(2) # Wait for connection and subscriptions

    # Example: Send an instant action to initialize position of robot 0
    print("\nSending initPosition action...")
    send_instant_action(
        robot_serial_numbers[0],
        "initPosition",
        params={"x": 1.0, "y": 1.0, "theta": 0.0, "mapId": config_data.settings.map_id}
    )
    time.sleep(1)

    # Example: Send a simple order to robot 0 to move between two points
    print("\nSending simple order...")
    node_list = [
        {'node_id': 'start_node', 'x': 1.0, 'y': 1.0},
        {'node_id': 'end_node', 'x': 5.0, 'y': 3.0}
    ]
    edge_list = [
        {'edge_id': 'move_edge', 'start_node_id': 'start_node', 'end_node_id': 'end_node'}
    ]
    send_order(robot_serial_numbers[0], "order_" + str(uuid.uuid4()), node_list, edge_list)
    time.sleep(1)


    # --- Setup and Run Visualization ---
    print("Setting up visualization...")

    # --- >> ADD: Create the figure and axes HERE << ---
    fig, ax = plt.subplots()

    # Use FuncAnimation for real-time updates
    # Now 'fig' is a valid Figure object when passed to FuncAnimation
    ani = animation.FuncAnimation(fig, update_plot, init_func=init_plot,
                                  interval=100, blit=True, cache_frame_data=False)

    plt.show() # Display the plot
    
    # --- Cleanup ---
    print("Stopping MQTT loop...")
    if mqtt_client:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
    print("Exiting.")
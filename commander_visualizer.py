# commander_visualizer.py

import paho.mqtt.client as mqtt
import json
import time
import uuid
import datetime
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

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

dropped_package_locations = [] # List to store (x, y) of dropped packages
drop_plot_object = None # Matplotlib plot object for dropped packages

path_plot_object = None 

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
     global dropped_package_locations # Allow modification

     if not mqtt_client:
        print("MQTT client not connected.")
        return

     base_topic = base_topics.get(serial_number)
     if not base_topic:
        print(f"Could not find base topic for {serial_number}")
        return

     nodes = []
     edges = []
     sequence_id_counter = 0 # Reset for each order

     # --- >> PROCESS NODES WITH ACTIONS << ---
     for n_data in nodes_data:
          sequence_id_counter += 1
          node_actions = []
          # Check if actions are defined for this node
          for act_data in n_data.get('actions', []):
               action_params = [ActionParameter(key=k, value=ActionParameterValue(v))
                                for k, v in act_data.get('action_parameters', {}).items()]
               node_actions.append(Action(
                    action_type=act_data['action_type'],
                    action_id=act_data.get('action_id', str(uuid.uuid4())), # Generate ID if missing
                    blocking_type=act_data.get('blocking_type', BlockingType.HARD),
                    action_parameters=action_params,
                    action_description=act_data.get('action_description')
               ))
               # --- If it's a dropOff action, record location for visualization ---
               if act_data['action_type'] == 'dropOff':
                    drop_loc = (n_data['x'], n_data['y'])
                    if drop_loc not in dropped_package_locations:
                         print(f"VISUALIZER: Recording drop-off location for {serial_number} at {drop_loc}")
                         dropped_package_locations.append(drop_loc)
               # --- End dropOff recording ---

          # Create NodePosition object
          node_pos = NodePosition(
               x=n_data['x'],
               y=n_data['y'],
               map_id=config_data.settings.map_id
               # Add theta, deviations etc. if needed
          )

          # Create the Node object
          nodes.append(Node(
               node_id=n_data['node_id'],
               sequence_id=sequence_id_counter,
               released=n_data.get('released', True), # Default to True
               node_description=n_data.get('node_description'),
               node_position=node_pos,
               actions=node_actions # Add the parsed/created actions
          ))
     # --- >> END NODE PROCESSING << ---

     # --- >> PROCESS EDGES (assuming no actions on edges for now) << ---
     for e_data in edges_data:
          sequence_id_counter += 1
          edges.append(Edge(
               edge_id=e_data['edge_id'],
               sequence_id=sequence_id_counter,
               start_node_id=e_data['start_node_id'],
               end_node_id=e_data['end_node_id'],
               released=e_data.get('released', True),
               actions=[] # Add edge action parsing if needed later
               # Add maxSpeed etc. if needed
          ))
     # --- >> END EDGE PROCESSING << ---

     # Create the final Order object
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

     # Send the order
     topic = f"{base_topic}/order"
     payload = json.dumps(order_cmd.to_dict(), indent=4) # Added indent for readability

     print(f"Sending order to {topic}") # Payload might be too long for console
     result = mqtt_client.publish(topic, payload=payload, qos=1)
     print(f"Publish result for order {order_id}: {result.rc}")

# --- Visualization Functions ---
def init_plot():
    """Initializes the Matplotlib plot axes and elements."""
    global drop_plot_object, path_plot_object
    ax.set_xlabel("X Coordinate")
    ax.set_ylabel("Y Coordinate")
    ax.set_title("Robot Positions & Path")
    ax.grid(True)
    ax.set_xlim(-1, 7)
    ax.set_ylim(-1, 7)
    ax.set_aspect('equal', adjustable='box')

    initial_artists = []

    # Path Visualization
    path_x = [node['x'] for node in node_data_template]
    path_y = [node['y'] for node in node_data_template]
    path_line, = ax.plot(path_x, path_y, marker='s',
                         linestyle='--', color='grey', linewidth=1, label='Path') # Label added once here
    path_plot_object = path_line
    initial_artists.append(path_plot_object)

    # Robot plots
    robot_handles = [] # Store handles for legend
    for serial in robot_serial_numbers:
        scatter, = ax.plot([], [], marker='o', linestyle='', markersize=8, label=serial) # Label per robot instance
        scatter_plots[serial] = scatter
        text = ax.text(0, 0, serial[-2:], fontsize=8, ha='center', va='bottom')
        text_labels[serial] = text
        initial_artists.extend([scatter, text])
        robot_handles.append(scatter) # Collect scatter plot handles

    # Plot for Dropped Packages
    drop_plot, = ax.plot([], [], marker='x', color='red', linestyle='', markersize=7, label='DropOff') # Label added once here
    drop_plot_object = drop_plot
    initial_artists.append(drop_plot_object)

    ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1), borderaxespad=0.)

    return initial_artists

def update_plot(frame):
    """Called periodically by FuncAnimation to update the plot."""
    updates = []
    # Update robots
    for serial, plot_obj in scatter_plots.items():
        if serial in robot_positions:
            x, y, theta = robot_positions[serial]
            plot_obj.set_data([x], [y])
            text_labels[serial].set_position((x, y + 0.2))
            updates.append(plot_obj)
            updates.append(text_labels[serial])
        else:
             plot_obj.set_data([], [])
             text_labels[serial].set_position((0, -1000))

    # Update Dropped Packages
    if drop_plot_object is not None and dropped_package_locations:
         drop_x = [loc[0] for loc in dropped_package_locations]
         drop_y = [loc[1] for loc in dropped_package_locations]
         drop_plot_object.set_data(drop_x, drop_y)
         updates.append(drop_plot_object)

    # The path plot (path_plot_object) is static, so it doesn't need to be returned here for blitting updates.
    # If blit=True causes issues, you might need to return all artists including path_plot_object.

    return updates

# --- Main Execution ---
if __name__ == "__main__":
    print("Starting Commander and Visualizer...")
    setup_mqtt()

    if not mqtt_client:
        print("Exiting due to MQTT connection failure.")
        exit()

    print("MQTT Client Setup Complete.")
    time.sleep(2)

    # --- >> REVERT to Square Path Definition << ---
    start_pos = (0.0, 0.0)
    station1_pos = (5.0, 0.0) # Station 1 at corner
    corner2_pos = (5.0, 5.0)
    station2_pos = (0.0, 5.0) # Station 2 at corner

    # Define Nodes for the square path
    node_data_template = [
        {'node_id': 'N_Start', 'x': start_pos[0], 'y': start_pos[1], 'actions': []},
        {'node_id': 'N_Station1', 'x': station1_pos[0], 'y': station1_pos[1], 'actions': [
            {'action_type': 'dropOff', 'action_id': 'act_drop1', 'blocking_type': BlockingType.HARD}
        ]},
        {'node_id': 'N_Corner2', 'x': corner2_pos[0], 'y': corner2_pos[1], 'actions': []},
        {'node_id': 'N_Station2', 'x': station2_pos[0], 'y': station2_pos[1], 'actions': [
            {'action_type': 'dropOff', 'action_id': 'act_drop2', 'blocking_type': BlockingType.HARD}
        ]},
         # Close the loop by going back to the start coordinates
        {'node_id': 'N_EndLoop', 'x': start_pos[0], 'y': start_pos[1], 'actions': []}
    ]

    # Define Edges for the square path
    edge_data_template = [
        {'edge_id': 'E_Start_S1', 'start_node_id': 'N_Start', 'end_node_id': 'N_Station1'},
        {'edge_id': 'E_S1_C2', 'start_node_id': 'N_Station1', 'end_node_id': 'N_Corner2'},
        {'edge_id': 'E_C2_S2', 'start_node_id': 'N_Corner2', 'end_node_id': 'N_Station2'},
        {'edge_id': 'E_S2_End', 'start_node_id': 'N_Station2', 'end_node_id': 'N_EndLoop'} # Edge back to start
    ]
    # --- >> END Path Definition Revert << ---

    # --- Send Commands Loop ---
    for index, serial in enumerate(robot_serial_numbers):
        print(f"\n--- Preparing commands for Robot: {serial} ---")

        # 1. Initialize Position (Stagger start slightly)
        init_x = start_pos[0] + index * 0.2 # Small stagger X
        init_y = start_pos[1] + index * 0.2 # Small stagger Y
        print(f"Sending initPosition action to {serial} at ({init_x:.1f}, {init_y:.1f})...")
        send_instant_action(
            serial,
            "initPosition",
            params={"x": init_x, "y": init_y, "theta": 0.0, "mapId": config_data.settings.map_id}
        )
        time.sleep(0.5)

        # 2. Create and Send the Order (Uses the square path templates now)
        order_id = f"order_square_{serial}_{str(uuid.uuid4())[:4]}"
        print(f"Sending square path order ({order_id}) to {serial}...")
        # Assuming send_order is correctly modified to handle node actions
        send_order(serial, order_id, node_data_template, edge_data_template)
        time.sleep(1)
    # --- End Send Commands Loop ---

    # --- Setup and Run Visualization ---
    print("Setting up visualization...")
    fig, ax = plt.subplots()
    # Note: init_plot now uses node_data_template defined above to draw the path
    ani = animation.FuncAnimation(fig, update_plot, init_func=init_plot,
                                  interval=100, blit=True, cache_frame_data=False)
    plt.show()
    
    # --- Cleanup ---
    print("Stopping MQTT loop...")
    if mqtt_client:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
    print("Exiting.")
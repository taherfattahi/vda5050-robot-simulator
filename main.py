import asyncio
import datetime
import json
import time
import random
import paho.mqtt.client as mqtt
from typing import Dict, List, Optional, Any, Tuple

import config
import mqtt_utils
import utils
from protocol.vda5050_common import AgvPosition
from protocol.vda_2_0_0.vda5050_2_0_0_connection import Connection, ConnectionState
from protocol.vda_2_0_0.vda5050_2_0_0_state import State, NodeState, EdgeState, ActionState, BatteryState, SafetyState, OperatingMode, ActionStatus, EStop
from protocol.vda_2_0_0.vda5050_2_0_0_visualization import Visualization
from protocol.vda_2_0_0.vda5050_2_0_0_action import Action, ActionParameter, ActionParameterValue, BlockingType
from protocol.vda_2_0_0.vda5050_2_0_0_order import Order
from protocol.vda_2_0_0.vda5050_2_0_0_instant_actions import InstantActions

class VehicleSimulator:
    """Simulates a VDA5050 vehicle"""
    def __init__(self, config_data: config.Config):
        self.config = config_data
        
        # Generate base MQTT topic
        base_topic = mqtt_utils.generate_vda_mqtt_base_topic(
            self.config.mqtt_broker.vda_interface,
            self.config.vehicle.vda_version,
            self.config.vehicle.manufacturer,
            self.config.vehicle.serial_number
        )
        
        # Connection
        self.connection_topic = f"{base_topic}/connection"
        self.connection = Connection(
            header_id=0,
            timestamp=utils.get_timestamp(),
            version=self.config.vehicle.vda_full_version,
            manufacturer=self.config.vehicle.manufacturer,
            serial_number=self.config.vehicle.serial_number,
            connection_state=ConnectionState.CONNECTION_BROKEN
        )
        
        # State
        self.state_topic = f"{base_topic}/state"
        random_x = random.random() * 5.0 - 2.5
        random_y = random.random() * 5.0 - 2.5
        agv_position = AgvPosition(
            x=random_x,
            y=random_y,
            position_initialized=True,
            theta=0.0,
            map_id=self.config.settings.map_id,
            deviation_range=None,
            map_description=None,
            localization_score=None
        )
        
        self.state = State(
            header_id=0,
            timestamp=utils.get_timestamp(),
            version=self.config.vehicle.vda_full_version,
            manufacturer=self.config.vehicle.manufacturer,
            serial_number=self.config.vehicle.serial_number,
            driving=False,
            distance_since_last_node=None,
            operating_mode=OperatingMode.AUTOMATIC,
            node_states=[],
            edge_states=[],
            last_node_id="",
            order_id="",
            order_update_id=0,
            last_node_sequence_id=0,
            action_states=[],
            information=[],
            loads=[],
            errors=[],
            battery_state=BatteryState(
                battery_charge=0.0,
                battery_voltage=None,
                battery_health=None,
                charging=False,
                reach=None
            ),
            safety_state=SafetyState(
                e_stop=EStop.NONE,
                field_violation=False
            ),
            paused=None,
            new_base_request=None,
            agv_position=agv_position,
            velocity=None,
            zone_set_id=None
        )
        
        # Visualization
        self.visualization_topic = f"{base_topic}/visualization"
        self.visualization = Visualization(
            header_id=0,
            timestamp=utils.get_timestamp(),
            version=self.config.vehicle.vda_full_version,
            manufacturer=self.config.vehicle.manufacturer,
            serial_number=self.config.vehicle.serial_number,
            agv_position=agv_position,
            velocity=None
        )
        
        # Order and Instant Actions
        self.order = None
        self.instant_actions = None
        self.action_start_time = None

    def run_action(self, action: Action) -> None:
        """Execute an action"""
        action_state_index = None
        for i, a_state in enumerate(self.state.action_states):
            if a_state.action_id == action.action_id:
                action_state_index = i
                break

        if action_state_index is not None:
            self.state.action_states[action_state_index].action_status = ActionStatus.RUNNING
            
            if action.action_type == "initPosition":
                # Find parameters
                x_param = None
                y_param = None
                theta_param = None
                map_id_param = None
                
                for param in action.action_parameters:
                    if param.key == "x":
                        x_param = param.value
                    elif param.key == "y":
                        y_param = param.value
                    elif param.key == "theta":
                        theta_param = param.value
                    elif param.key == "mapId":
                        map_id_param = param.value
                
                # Extract values
                x_float = float(x_param.value) if x_param else 0.0
                y_float = float(y_param.value) if y_param else 0.0
                theta_float = float(theta_param.value) if theta_param else 0.0
                map_id_string = str(map_id_param.value) if map_id_param else ""
                
                # Update position
                self.state.agv_position = AgvPosition(
                    x=x_float,
                    y=y_float,
                    position_initialized=True,
                    theta=theta_float,
                    map_id=map_id_string,
                    deviation_range=None,
                    map_description=None,
                    localization_score=None
                )
                self.visualization.agv_position = self.state.agv_position
            
            # Mark action as finished
            self.state.action_states[action_state_index].action_status = ActionStatus.FINISHED
    
    async def publish_connection(self, mqtt_client: mqtt.Client) -> None:
        """Publish connection state"""
        # First publish ConnectionBroken state
        json_connection_broken = json.dumps(self.connection.to_dict())
        await mqtt_utils.mqtt_publish(mqtt_client, self.connection_topic, json_connection_broken)

        # Wait for 1 second
        await asyncio.sleep(1)
        
        # Update to Online state
        self.connection.header_id += 1
        self.connection.timestamp = utils.get_timestamp()
        self.connection.connection_state = ConnectionState.ONLINE
        json_connection_online = json.dumps(self.connection.to_dict())
        await mqtt_utils.mqtt_publish(mqtt_client, self.connection_topic, json_connection_online)
    
    async def publish_visualization(self, mqtt_client: mqtt.Client) -> None:
        """Publish visualization data"""
        self.visualization.header_id += 1
        self.visualization.timestamp = utils.get_timestamp()
        json_visualization = json.dumps(self.visualization.to_dict())
        await mqtt_utils.mqtt_publish(mqtt_client, self.visualization_topic, json_visualization)
    
    async def publish_state(self, mqtt_client: mqtt.Client) -> None:
        """Publish state data"""
        self.state.header_id += 1
        self.state.timestamp = utils.get_timestamp()
        json_state = json.dumps(self.state.to_dict())
        await mqtt_utils.mqtt_publish(mqtt_client, self.state_topic, json_state)
    
    def instant_actions_accept_procedure(self, instant_action_request: InstantActions) -> None:
        """Process incoming instant actions"""
        # TODO: Add validation
        
        self.instant_actions = instant_action_request
        
        # Add instant actions to action states
        for instant_action in self.instant_actions.instant_actions:
            action_state = ActionState(
                action_id=instant_action.action_id,
                action_status=ActionStatus.WAITING,
                action_type=instant_action.action_type,
                action_description=None,
                result_description=None
            )
            self.state.action_states.append(action_state)
    
    def order_accept_procedure(self, order_request: Order) -> None:
        """Process incoming order request"""
        if order_request.order_id != self.state.order_id:
            # Empty string (""), if no previous orderId is available
            if self.state.order_id == "":
                self.order_accept(order_request)
                return
            
            # Check action states
            if len(self.state.node_states) == 0 and len(self.state.edge_states) == 0:
                # Delete action states
                self.state.action_states = []
                self.order_accept(order_request)
                return
            else:
                self.order_reject("There is order_state or edge_state in state")
                return
        else:
            if order_request.order_update_id > self.state.order_update_id:
                if len(self.state.node_states) > 0 and len(self.state.edge_states) == 0:
                    # Delete action states
                    self.state.action_states = []
                    self.order_accept(order_request)
                    return
                else:
                    self.order_reject("There is order_state or edge_state in state1")
                    return
            else:
                self.order_reject("Order update id is lower")
                return
    
    def order_accept(self, order_request: Order) -> None:
        """Accept an order"""
        print(f"SIMULATOR ({self.config.vehicle.serial_number}): Accepting order {order_request.order_id}") # Added serial
        self.order = order_request

        # Find the first node in the order based on sequence ID.
        # VDA5050 sequence IDs usually start at 1. Find the node with the lowest sequence ID.
        first_node = None
        if self.order.nodes:
             # Sort nodes by sequence ID just in case they aren't ordered in the request
             sorted_nodes = sorted(self.order.nodes, key=lambda n: n.sequence_id)
             first_node = sorted_nodes[0]

        if first_node:
             # Set initial last_node and sequence to the *first* node of the new order.
             # This makes the state ready to find the *next* node (the second one)
             # or execute actions on the first node.
             self.state.last_node_id = first_node.node_id
             self.state.last_node_sequence_id = first_node.sequence_id
             print(f"SIMULATOR ({self.config.vehicle.serial_number}): Initializing state to first node: ID={self.state.last_node_id}, Seq={self.state.last_node_sequence_id}")
        else:
             # Handle case where order has no nodes (edge case)
             self.state.last_node_id = ""
             self.state.last_node_sequence_id = 0
             print(f"SIMULATOR ({self.config.vehicle.serial_number}): Warning - Accepted order has no nodes.")

        # Set orderId and orderUpdateId *after* determining the starting node info
        self.state.order_id = self.order.order_id
        self.state.order_update_id = self.order.order_update_id

        # Delete old states
        self.state.action_states = []
        self.state.node_states = []
        self.state.edge_states = []
        
        # Set nodeStates, edgeStates, and actionStates
        for node in self.order.nodes:
            node_state = NodeState(
                node_id=node.node_id,
                sequence_id=node.sequence_id,
                released=node.released,
                node_description=node.node_description,
                node_position=node.node_position
            )
            self.state.node_states.append(node_state)
            
            for action in node.actions:
                action_state = ActionState(
                    action_id=action.action_id,
                    action_type=action.action_type,
                    action_description=action.action_description,
                    action_status=ActionStatus.WAITING,
                    result_description=None
                )
                self.state.action_states.append(action_state)
        
        for edge in self.order.edges:
            edge_state = EdgeState(
                edge_id=edge.edge_id,
                sequence_id=edge.sequence_id,
                released=edge.released,
                edge_description=edge.edge_description,
                trajectory=None
            )
            self.state.edge_states.append(edge_state)
            
            for action in edge.actions:
                action_state = ActionState(
                    action_id=action.action_id,
                    action_type=action.action_type,
                    action_description=action.action_description,
                    action_status=ActionStatus.WAITING,
                    result_description=None
                )
                self.state.action_states.append(action_state)
    
    def order_reject(self, reason: str) -> None:
        """Reject an order"""
        print(f"Order reject: {reason}")
    
    def state_iterate(self) -> None:
        """Update state based on simulation logic"""
        
        # Inside state_iterate, after action time check
        # print(f"DEBUG ({self.config.vehicle.serial_number}): Start state_iterate. Order ID: {self.state.order_id}, Update ID: {self.state.order_update_id}")
        # print(f"DEBUG ({self.config.vehicle.serial_number}): Last Node ID: {self.state.last_node_id}, Last Seq ID: {self.state.last_node_sequence_id}")
        # print(f"DEBUG ({self.config.vehicle.serial_number}): Current Node States: {[ns.node_id for ns in self.state.node_states]}")
        # print(f"DEBUG ({self.config.vehicle.serial_number}): Current Edge States: {[es.edge_id for es in self.state.edge_states]}")
        # print(f"DEBUG ({self.config.vehicle.serial_number}): Current Position: ({self.state.agv_position.x:.2f}, {self.state.agv_position.y:.2f})")

        # Check action time
        if self.action_start_time is not None:
            current_time = datetime.datetime.utcnow()
            action_end_time = self.action_start_time + datetime.timedelta(seconds=self.config.settings.action_time)
            if current_time < action_end_time:
                return

        # Start instant action
        if self.instant_actions is not None:
            # Get instant actions
            instant_actions = self.instant_actions.instant_actions
            for action in instant_actions:
                # Find the action state
                for action_state in self.state.action_states:
                    if action_state.action_id == action.action_id and action_state.action_status == ActionStatus.WAITING:
                        self.run_action(action)
                        break
        
        # Check order
        if self.order is None:
            return
        
        # Find last node index
        order_last_node_index = None
        for i, node in enumerate(self.order.nodes):
            if node.sequence_id == self.state.last_node_sequence_id:
                order_last_node_index = i
                break
        
        if order_last_node_index is not None:
            # Get last node actions
            check_actions = self.order.nodes[order_last_node_index].actions
            
            if check_actions:
                # Process actions
                for action_state in self.state.action_states:
                    for check_action in check_actions:
                        if action_state.action_id == check_action.action_id and action_state.action_status == ActionStatus.WAITING:
                            print(f"Action type: {action_state.action_type}")
                            action_state.action_status = ActionStatus.FINISHED
                            self.action_start_time = datetime.datetime.utcnow()
                            return
        
        # Check vehicle position
        if self.state.agv_position is None:
            return
        
        # Remove last node if it's the only one left
        if len(self.state.node_states) == 1:
            self.state.node_states.pop(0)
            return
        
        if len(self.state.node_states) == 0:
            return
        
        vehicle_position = self.state.agv_position
        last_node_index = None
        
        # Find index of last node
        for i, node_state in enumerate(self.state.node_states):
            if node_state.sequence_id == self.state.last_node_sequence_id:
                last_node_index = i
                break
        
        # Last node is not found or is the last in the list
        if last_node_index is None or last_node_index >= len(self.state.node_states) - 1:
            return
        
        next_node = self.state.node_states[last_node_index + 1]
        
        if next_node.node_position is None:
            return
        
        next_node_position = next_node.node_position
        
        # Update position
        updated_vehicle_position = utils.iterate_position(
            vehicle_position.x,
            vehicle_position.y,
            next_node_position.x,
            next_node_position.y,
            self.config.settings.speed
        )
        
        self.state.agv_position.x = updated_vehicle_position[0]
        self.state.agv_position.y = updated_vehicle_position[1]
        self.state.agv_position.theta = updated_vehicle_position[2]
        
        self.visualization.agv_position = self.state.agv_position
        
        # Check if we've reached the next node
        distance_to_next_node = utils.get_distance(
            vehicle_position.x,
            vehicle_position.y,
            next_node_position.x,
            next_node_position.y
        )
        
        if distance_to_next_node < self.config.settings.speed + 0.1:
            if self.state.node_states:
                self.state.node_states.pop(0)
            if self.state.edge_states:
                self.state.edge_states.pop(0)
            
            self.state.last_node_id = next_node.node_id
            self.state.last_node_sequence_id = next_node.sequence_id

class MQTTClient:
    """MQTT client for VDA5050 communication"""
    def __init__(self, config_data: config.Config):
        self.config = config_data
        self.client = None
        self.base_topic = mqtt_utils.generate_vda_mqtt_base_topic(
            self.config.mqtt_broker.vda_interface,
            self.config.vehicle.vda_version,
            self.config.vehicle.manufacturer,
            self.config.vehicle.serial_number
        )
        self.message_queue = None # Will be created in connect
        self.loop = None # <<< ADD: To store the asyncio loop
    
    def on_connect(self, client, userdata, flags, rc, properties=None):
        """Callback when connected to MQTT broker"""
        print(f"Connected with result code {rc}")
        
        # Subscribe to topics
        topics = [
            f"{self.base_topic}/order",
            f"{self.base_topic}/instantActions"
        ]
        for topic in topics:
            client.subscribe(topic, qos=1)
            print(f"SIM ({self.config.vehicle.serial_number}): Subscribed to {topic}")
    
    def on_message(self, client, userdata, msg):
        """Callback when message is received (Runs in MQTT Thread)"""
        topic = msg.topic
        topic_type = utils.get_topic_type(topic)
        payload = msg.payload.decode('utf-8')
        print(f"SIM ({self.config.vehicle.serial_number}): Received raw message on {topic_type}") # Added serial for clarity

        if self.loop is None or self.message_queue is None:
             print(f"SIM ({self.config.vehicle.serial_number}): Error - Loop or Queue not initialized in on_message.")
             return

        try:
            if topic_type == "order":
                order_data = json.loads(payload)
                order = Order.from_dict(order_data)
                # --- V V V CHANGE V V V ---
                # Safely put item onto the asyncio queue from this thread
                self.loop.call_soon_threadsafe(self.message_queue.put_nowait, ("order", order))
                # --- ^ ^ ^ CHANGE ^ ^ ^ ---
                print(f"SIM ({self.config.vehicle.serial_number}): Queued order {order.order_id}")

            elif topic_type == "instantActions":
                instant_actions_data = json.loads(payload)
                instant_actions = InstantActions.from_dict(instant_actions_data)
                 # --- V V V CHANGE V V V ---
                # Safely put item onto the asyncio queue from this thread
                self.loop.call_soon_threadsafe(self.message_queue.put_nowait, ("instantActions", instant_actions))
                # --- ^ ^ ^ CHANGE ^ ^ ^ ---
                print(f"SIM ({self.config.vehicle.serial_number}): Queued instant actions")

        except Exception as e:
            print(f"SIM ({self.config.vehicle.serial_number}): Error processing message on {topic}: {e}")

    def connect(self):
        """Connect to MQTT broker"""
        opts = mqtt_utils.mqtt_create_opts()
        # Use unique client ID for each simulator instance
        client_id_suffix = self.config.vehicle.serial_number.replace(" ", "_") # Make suffix safe for client ID
        self.client = mqtt.Client(client_id=f"{opts['client_id']}-{client_id_suffix}", protocol=mqtt.MQTTv5)


        # Set callbacks
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        # --- V V V CHANGE V V V ---
        # Get the running asyncio event loop
        self.loop = asyncio.get_running_loop()
        # Create the asyncio queue (needs the loop)
        self.message_queue = asyncio.Queue()
        # --- ^ ^ ^ CHANGE ^ ^ ^ ---

        # Connect to broker
        self.client.connect(
            host=self.config.mqtt_broker.host,
            port=int(self.config.mqtt_broker.port)
        )

        # Start the loop (in a separate thread)
        self.client.loop_start()

        # Return the queue for the main async logic
        return self.client, self.message_queue

async def subscribe_vda_messages(vehicle_simulator, mqtt_client, message_queue):
    """Process messages from MQTT broker"""
    while True:
        try:
            # Get message from queue
            message_type, message_data = await message_queue.get()
            
            if message_type == "order":
                vehicle_simulator.order_accept_procedure(message_data)
            elif message_type == "instantActions":
                vehicle_simulator.instant_actions_accept_procedure(message_data)
            
            # Mark task as done
            message_queue.task_done()
        
        except Exception as e:
            print(f"Error processing message: {e}")
            await asyncio.sleep(1)

async def publish_vda_messages(vehicle_simulator, mqtt_client, state_frequency, visualization_frequency):
    """Publish messages to MQTT broker"""
    # First publish connection state
    await vehicle_simulator.publish_connection(mqtt_client)
    
    # Calculate tick time (50ms)
    tick_time = 0.05
    counter_state = 0
    counter_visualization = 0
    
    while True:
        # Update vehicle state
        vehicle_simulator.state_iterate()
        
        
        # Check if it's time to publish state
        counter_state += 1
        if counter_state * tick_time > 1.0 / state_frequency:
            counter_state = 0
            await vehicle_simulator.publish_state(mqtt_client)
        
        # Check if it's time to publish visualization
        counter_visualization += 1
        if counter_visualization * tick_time > 1.0 / visualization_frequency:
            counter_visualization = 0
            await vehicle_simulator.publish_visualization(mqtt_client)
        
        # Sleep for tick time
        await asyncio.sleep(tick_time)

async def main():
    """Main function"""
    # Load configuration
    config_data = config.get_config()
    
    # Create tasks for each robot
    tasks = []
    
    for robot_index in range(config_data.settings.robot_count):
        # Clone generic config
        vehicle_config = config_data
        
        # Rename robot serial number
        vehicle_config.vehicle.serial_number = f"{config_data.vehicle.serial_number}{robot_index}"
        
        # Create vehicle simulator
        vehicle_simulator = VehicleSimulator(vehicle_config)
        
        # Create MQTT client
        mqtt_handler = MQTTClient(vehicle_config)
        mqtt_client, message_queue = mqtt_handler.connect()
        
        # Create tasks
        subscribe_task = asyncio.create_task(
            subscribe_vda_messages(vehicle_simulator, mqtt_client, message_queue)
        )
        
        publish_task = asyncio.create_task(
            publish_vda_messages(
                vehicle_simulator,
                mqtt_client,
                config_data.settings.state_frequency,
                config_data.settings.visualization_frequency
            )
        )
        
        tasks.extend([subscribe_task, publish_task])
    
    # Wait for all tasks
    await asyncio.gather(*tasks)

if __name__ == "__main__":
    asyncio.run(main())
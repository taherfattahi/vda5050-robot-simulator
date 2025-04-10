import paho.mqtt.client as mqtt
import uuid
import json
from config import get_config

def mqtt_create_opts():
    """Create MQTT client options"""
    config = get_config()
    
    server_uri = f"tcp://{config.mqtt_broker.host}:{config.mqtt_broker.port}"
    client_id = str(uuid.uuid4())
    
    print(f"Creating client to: {server_uri}, client_id: {client_id}")
    
    return {
        "server_uri": server_uri,
        "client_id": client_id
    }

async def mqtt_publish(mqtt_client, topic, data):
    """Publish message to MQTT broker"""
    json_data = json.loads(data)
    payload = json.dumps(json_data).encode('utf-8')
    result = mqtt_client.publish(topic, payload=payload, qos=1)
    result.wait_for_publish()
    return result

def generate_vda_mqtt_base_topic(vda_interface, vda_version, manufacturer, serial_number):
    """Generate base topic for VDA5050 communication"""
    vda5050_base_topic = f"{vda_interface}/{vda_version}/{manufacturer}/{serial_number}"
    return vda5050_base_topic
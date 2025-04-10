import datetime
import math

def get_timestamp():
    """Get current timestamp in ISO8601 format"""
    now = datetime.datetime.utcnow()
    return now.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"

def get_topic_type(path):
    """Extract topic type from MQTT topic path"""
    parts = path.rsplit('/', 1)
    if len(parts) > 1:
        return parts[1]
    return path

def check_deviation_range(node_x, node_y, vehicle_x, vehicle_y, deviation_range):
    """Check if vehicle is within deviation range of node"""
    distance = math.sqrt((node_x - vehicle_x) ** 2 + (node_y - vehicle_y) ** 2)
    return distance <= deviation_range

def iterate_position(current_x, current_y, target_x, target_y, speed):
    """Calculate next position when moving towards target"""
    angle = math.atan2(target_y - current_y, target_x - current_x)
    next_x = current_x + speed * math.cos(angle)
    next_y = current_y + speed * math.sin(angle)
    
    return (next_x, next_y, angle)

def get_distance(x1, y1, x2, y2):
    """Calculate distance between two points"""
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
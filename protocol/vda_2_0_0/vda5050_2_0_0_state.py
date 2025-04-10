from dataclasses import dataclass, field
from typing import List, Dict, Optional
from enum import Enum
from protocol.vda5050_common import AgvPosition, BoundingBoxReference, HeaderId, LoadDimensions, NodePosition, Trajectory, Velocity

class OperatingMode(str, Enum):
    AUTOMATIC = "AUTOMATIC"
    SEMIAUTOMATIC = "SEMIAUTOMATIC"
    MANUAL = "MANUAL"
    SERVICE = "SERVICE"
    TEACHIN = "TEACHIN"

class ActionStatus(str, Enum):
    WAITING = "WAITING"  # Action was received but node not yet reached
    INITIALIZING = "INITIALIZING"  # Action was triggered, preparatory measures are initiated
    RUNNING = "RUNNING"  # The action is running
    PAUSED = "PAUSED"  # The action is paused
    FINISHED = "FINISHED"  # The action is finished
    FAILED = "FAILED"  # Action could not be finished

class EStop(str, Enum):
    AUTOACK = "AUTOACK"  # Auto-acknowledgeable e-stop is activated
    MANUAL = "MANUAL"  # E-stop has to be acknowledged manually at the vehicle
    REMOTE = "REMOTE"  # Facility e-stop has to be acknowledged remotely
    NONE = "NONE"  # No e-stop activated

class ErrorLevel(str, Enum):
    WARNING = "WARNING"  # AGV is ready to start
    FATAL = "FATAL"  # AGV is not in running condition, user intervention required

class InfoLevel(str, Enum):
    INFO = "INFO"  # Used for visualization
    DEBUG = "DEBUG"  # Used for debugging

@dataclass
class NodeState:
    """Information about a node that the AGV still has to drive over"""
    node_id: str
    sequence_id: int
    released: bool
    node_description: Optional[str] = None
    node_position: Optional[NodePosition] = None
    
    def to_dict(self) -> Dict:
        result = {
            "nodeId": self.node_id,
            "sequenceId": self.sequence_id,
            "released": self.released
        }
        if self.node_description is not None:
            result["nodeDescription"] = self.node_description
        if self.node_position is not None:
            result["nodePosition"] = self.node_position.to_dict()
        return result

@dataclass
class EdgeState:
    """Information about an edge that the AGV still has to drive over"""
    edge_id: str
    sequence_id: int
    released: bool
    edge_description: Optional[str] = None
    trajectory: Optional[Trajectory] = None
    
    def to_dict(self) -> Dict:
        result = {
            "edgeId": self.edge_id,
            "sequenceId": self.sequence_id,
            "released": self.released
        }
        if self.edge_description is not None:
            result["edgeDescription"] = self.edge_description
        if self.trajectory is not None:
            result["trajectory"] = self.trajectory.to_dict()
        return result

@dataclass
class ActionState:
    """Information about an action"""
    action_id: str
    action_status: ActionStatus
    action_type: Optional[str] = None
    action_description: Optional[str] = None
    result_description: Optional[str] = None
    
    def to_dict(self) -> Dict:
        result = {
            "actionId": self.action_id,
            "actionStatus": self.action_status.value
        }
        if self.action_type is not None:
            result["actionType"] = self.action_type
        if self.action_description is not None:
            result["actionDescription"] = self.action_description
        if self.result_description is not None:
            result["resultDescription"] = self.result_description
        return result

@dataclass
class Load:
    """Load object that describes the load if the AGV has information about it"""
    load_id: Optional[str] = None
    load_type: Optional[str] = None
    load_position: Optional[str] = None
    bounding_box_reference: Optional[BoundingBoxReference] = None
    load_dimensions: Optional[LoadDimensions] = None
    weight: Optional[float] = None
    
    def to_dict(self) -> Dict:
        result = {}
        if self.load_id is not None:
            result["loadId"] = self.load_id
        if self.load_type is not None:
            result["loadType"] = self.load_type
        if self.load_position is not None:
            result["loadPosition"] = self.load_position
        if self.bounding_box_reference is not None:
            result["boundingBoxReference"] = self.bounding_box_reference.to_dict()
        if self.load_dimensions is not None:
            result["loadDimensions"] = self.load_dimensions.to_dict()
        if self.weight is not None:
            result["weight"] = self.weight
        return result

@dataclass
class BatteryState:
    """Contains all battery-related information"""
    battery_charge: float
    charging: bool
    battery_voltage: Optional[float] = None
    battery_health: Optional[int] = None
    reach: Optional[float] = None
    
    def to_dict(self) -> Dict:
        result = {
            "batteryCharge": self.battery_charge,
            "charging": self.charging
        }
        if self.battery_voltage is not None:
            result["batteryVoltage"] = self.battery_voltage
        if self.battery_health is not None:
            result["batteryHealth"] = self.battery_health
        if self.reach is not None:
            result["reach"] = self.reach
        return result

@dataclass
class ErrorReference:
    """Object that holds the error reference as key-value pairs"""
    reference_key: str
    reference_value: str
    
    def to_dict(self) -> Dict:
        return {
            "referenceKey": self.reference_key,
            "referenceValue": self.reference_value
        }

@dataclass
class Error:
    """An error object"""
    error_type: str
    error_level: ErrorLevel
    error_references: List[ErrorReference]
    error_description: Optional[str] = None
    
    def to_dict(self) -> Dict:
        result = {
            "errorType": self.error_type,
            "errorLevel": self.error_level.value,
            "errorReferences": [ref.to_dict() for ref in self.error_references]
        }
        if self.error_description is not None:
            result["errorDescription"] = self.error_description
        return result

@dataclass
class InfoReference:
    """Object that holds the info reference as key-value pairs"""
    reference_key: str
    reference_value: str
    
    def to_dict(self) -> Dict:
        return {
            "referenceKey": self.reference_key,
            "referenceValue": self.reference_value
        }

@dataclass
class Information:
    """An information object"""
    info_type: str
    info_level: InfoLevel
    info_references: List[InfoReference]
    info_description: Optional[str] = None
    
    def to_dict(self) -> Dict:
        result = {
            "infoType": self.info_type,
            "infoLevel": self.info_level.value,
            "infoReferences": [ref.to_dict() for ref in self.info_references]
        }
        if self.info_description is not None:
            result["infoDescription"] = self.info_description
        return result

@dataclass
class SafetyState:
    """Object that holds information about the safety status"""
    e_stop: EStop
    field_violation: bool
    
    def to_dict(self) -> Dict:
        return {
            "eStop": self.e_stop.value,
            "fieldViolation": self.field_violation
        }

@dataclass
class State:
    """All encompassing state of the AGV"""
    header_id: HeaderId
    timestamp: str
    version: str
    manufacturer: str
    serial_number: str
    order_id: str
    order_update_id: int
    last_node_id: str
    last_node_sequence_id: int
    driving: bool
    operating_mode: OperatingMode
    node_states: List[NodeState]
    edge_states: List[EdgeState]
    action_states: List[ActionState]
    battery_state: BatteryState
    errors: List[Error]
    information: List[Information]
    loads: List[Load]
    safety_state: SafetyState
    agv_position: Optional[AgvPosition] = None
    velocity: Optional[Velocity] = None
    zone_set_id: Optional[str] = None
    paused: Optional[bool] = None
    new_base_request: Optional[bool] = None
    distance_since_last_node: Optional[float] = None
    
    def to_dict(self) -> Dict:
        result = {
            "headerId": self.header_id,
            "timestamp": self.timestamp,
            "version": self.version,
            "manufacturer": self.manufacturer,
            "serialNumber": self.serial_number,
            "orderId": self.order_id,
            "orderUpdateId": self.order_update_id,
            "lastNodeId": self.last_node_id,
            "lastNodeSequenceId": self.last_node_sequence_id,
            "driving": self.driving,
            "operatingMode": self.operating_mode.value,
            "nodeStates": [node.to_dict() for node in self.node_states],
            "edgeStates": [edge.to_dict() for edge in self.edge_states],
            "actionStates": [action.to_dict() for action in self.action_states],
            "batteryState": self.battery_state.to_dict(),
            "errors": [error.to_dict() for error in self.errors],
            "information": [info.to_dict() for info in self.information],
            "loads": [load.to_dict() for load in self.loads],
            "safetyState": self.safety_state.to_dict()
        }
        if self.agv_position is not None:
            result["agvPosition"] = self.agv_position.to_dict()
        if self.velocity is not None:
            result["velocity"] = self.velocity.to_dict()
        if self.zone_set_id is not None:
            result["zoneSetId"] = self.zone_set_id
        if self.paused is not None:
            result["paused"] = self.paused
        if self.new_base_request is not None:
            result["newBaseRequest"] = self.new_base_request
        if self.distance_since_last_node is not None:
            result["distanceSinceLastNode"] = self.distance_since_last_node
        return result
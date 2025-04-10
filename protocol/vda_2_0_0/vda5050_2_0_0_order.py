from dataclasses import dataclass
from typing import List, Dict, Optional
from enum import Enum
from protocol.vda5050_common import HeaderId, NodePosition, Trajectory
from protocol.vda_2_0_0.vda5050_2_0_0_action import Action

class OrientationType(str, Enum):
    GLOBAL = "GLOBAL"  # Relative to the global project specific map coordinate system
    TANGENTIAL = "TANGENTIAL"  # Tangential to the edge

@dataclass
class Node:
    """Node in the order graph"""
    node_id: str
    sequence_id: int
    released: bool
    actions: List[Action]
    node_description: Optional[str] = None
    node_position: Optional[NodePosition] = None
    
    def to_dict(self) -> Dict:
        result = {
            "nodeId": self.node_id,
            "sequenceId": self.sequence_id,
            "released": self.released,
            "actions": [action.to_dict() for action in self.actions]
        }
        if self.node_description is not None:
            result["nodeDescription"] = self.node_description
        if self.node_position is not None:
            result["nodePosition"] = self.node_position.to_dict()
        return result

@dataclass
class Edge:
    """Edge in the order graph"""
    edge_id: str
    sequence_id: int
    released: bool
    start_node_id: str
    end_node_id: str
    actions: List[Action]
    edge_description: Optional[str] = None
    max_speed: Optional[float] = None
    max_height: Optional[float] = None
    min_height: Optional[float] = None
    orientation: Optional[float] = None
    orientation_type: Optional[OrientationType] = None
    direction: Optional[str] = None
    rotation_allowed: Optional[bool] = None
    max_rotation_speed: Optional[float] = None
    length: Optional[float] = None
    trajectory: Optional[Trajectory] = None
    
    def to_dict(self) -> Dict:
        result = {
            "edgeId": self.edge_id,
            "sequenceId": self.sequence_id,
            "released": self.released,
            "startNodeId": self.start_node_id,
            "endNodeId": self.end_node_id,
            "actions": [action.to_dict() for action in self.actions]
        }
        if self.edge_description is not None:
            result["edgeDescription"] = self.edge_description
        if self.max_speed is not None:
            result["maxSpeed"] = self.max_speed
        if self.max_height is not None:
            result["maxHeight"] = self.max_height
        if self.min_height is not None:
            result["minHeight"] = self.min_height
        if self.orientation is not None:
            result["orientation"] = self.orientation
        if self.orientation_type is not None:
            result["orientationType"] = self.orientation_type.value
        if self.direction is not None:
            result["direction"] = self.direction
        if self.rotation_allowed is not None:
            result["rotationAllowed"] = self.rotation_allowed
        if self.max_rotation_speed is not None:
            result["maxRotationSpeed"] = self.max_rotation_speed
        if self.length is not None:
            result["length"] = self.length
        if self.trajectory is not None:
            result["trajectory"] = self.trajectory.to_dict()
        return result

@dataclass
class Order:
    """An order to be communicated from master control to the AGV"""
    header_id: HeaderId
    timestamp: str
    version: str
    manufacturer: str
    serial_number: str
    order_id: str
    order_update_id: int
    nodes: List[Node]
    edges: List[Edge]
    zone_set_id: Optional[str] = None
    
    def to_dict(self) -> Dict:
        result = {
            "headerId": self.header_id,
            "timestamp": self.timestamp,
            "version": self.version,
            "manufacturer": self.manufacturer,
            "serialNumber": self.serial_number,
            "orderId": self.order_id,
            "orderUpdateId": self.order_update_id,
            "nodes": [node.to_dict() for node in self.nodes],
            "edges": [edge.to_dict() for edge in self.edges]
        }
        if self.zone_set_id is not None:
            result["zoneSetId"] = self.zone_set_id
        return result
    
    @classmethod
    def from_dict(cls, data: Dict):
        # Import here to avoid circular imports
        from protocol.vda_2_0_0.vda5050_2_0_0_action import Action, ActionParameter, ActionParameterValue, BlockingType
        from protocol.vda5050_common import NodePosition
        
        # Process nodes
        nodes = []
        for node_data in data.get("nodes", []):
            node_actions = []
            for action_data in node_data.get("actions", []):
                action_params = []
                for param in action_data.get("actionParameters", []):
                    action_params.append(ActionParameter(
                        key=param["key"],
                        value=ActionParameterValue(param["value"])
                    ))
                
                node_actions.append(Action(
                    action_type=action_data["actionType"],
                    action_id=action_data["actionId"],
                    blocking_type=BlockingType(action_data["blockingType"]),
                    action_parameters=action_params,
                    action_description=action_data.get("actionDescription")
                ))
            
            node_position = None
            if "nodePosition" in node_data:
                pos = node_data["nodePosition"]
                node_position = NodePosition(
                    x=pos["x"],
                    y=pos["y"],
                    map_id=pos["mapId"],
                    theta=pos.get("theta"),
                    allowed_deviation_xy=pos.get("allowedDeviationXY"),
                    allowed_deviation_theta=pos.get("allowedDeviationTheta"),
                    map_description=pos.get("mapDescription")
                )
            
            nodes.append(Node(
                node_id=node_data["nodeId"],
                sequence_id=node_data["sequenceId"],
                released=node_data["released"],
                node_description=node_data.get("nodeDescription"),
                node_position=node_position,
                actions=node_actions
            ))
        
        # Process edges
        edges = []
        for edge_data in data.get("edges", []):
            edge_actions = []
            for action_data in edge_data.get("actions", []):
                action_params = []
                for param in action_data.get("actionParameters", []):
                    action_params.append(ActionParameter(
                        key=param["key"],
                        value=ActionParameterValue(param["value"])
                    ))
                
                edge_actions.append(Action(
                    action_type=action_data["actionType"],
                    action_id=action_data["actionId"],
                    blocking_type=BlockingType(action_data["blockingType"]),
                    action_parameters=action_params,
                    action_description=action_data.get("actionDescription")
                ))
            
            edges.append(Edge(
                edge_id=edge_data["edgeId"],
                sequence_id=edge_data["sequenceId"],
                released=edge_data["released"],
                start_node_id=edge_data["startNodeId"],
                end_node_id=edge_data["endNodeId"],
                edge_description=edge_data.get("edgeDescription"),
                max_speed=edge_data.get("maxSpeed"),
                max_height=edge_data.get("maxHeight"),
                min_height=edge_data.get("minHeight"),
                orientation=edge_data.get("orientation"),
                orientation_type=OrientationType(edge_data["orientationType"]) if "orientationType" in edge_data else None,
                direction=edge_data.get("direction"),
                rotation_allowed=edge_data.get("rotationAllowed"),
                max_rotation_speed=edge_data.get("maxRotationSpeed"),
                length=edge_data.get("length"),
                trajectory=None,  # TODO: implement trajectory parsing if needed
                actions=edge_actions
            ))
        
        return cls(
            header_id=data["headerId"],
            timestamp=data["timestamp"],
            version=data["version"],
            manufacturer=data["manufacturer"],
            serial_number=data["serialNumber"],
            order_id=data["orderId"],
            order_update_id=data["orderUpdateId"],
            zone_set_id=data.get("zoneSetId"),
            nodes=nodes,
            edges=edges
        )
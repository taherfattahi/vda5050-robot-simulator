from dataclasses import dataclass, field
from typing import List, Optional, Union

# Type alias
HeaderId = int

@dataclass
class AgvPosition:
    """Current position of the AGV on the map"""
    x: float
    y: float
    theta: float
    map_id: str
    position_initialized: bool
    map_description: Optional[str] = None
    localization_score: Optional[float] = None
    deviation_range: Optional[float] = None
    
    def to_dict(self):
        result = {
            "x": self.x,
            "y": self.y,
            "theta": self.theta,
            "mapId": self.map_id,
            "positionInitialized": self.position_initialized
        }
        if self.map_description is not None:
            result["mapDescription"] = self.map_description
        if self.localization_score is not None:
            result["localizationScore"] = self.localization_score
        if self.deviation_range is not None:
            result["deviationRange"] = self.deviation_range
        return result

@dataclass
class BoundingBoxReference:
    """Describes the loads position on the AGV in the vehicle coordinates"""
    x: float
    y: float
    z: float
    theta: Optional[float] = None
    
    def to_dict(self):
        result = {
            "x": self.x,
            "y": self.y,
            "z": self.z
        }
        if self.theta is not None:
            result["theta"] = self.theta
        return result

@dataclass
class ControlPoint:
    """Control point for trajectory"""
    x: float
    y: float
    weight: Optional[float] = None
    orientation: Optional[float] = None
    
    def to_dict(self):
        result = {
            "x": self.x,
            "y": self.y
        }
        if self.weight is not None:
            result["weight"] = self.weight
        if self.orientation is not None:
            result["orientation"] = self.orientation
        return result

@dataclass
class LoadDimensions:
    """Dimensions of the load's bounding box in meters"""
    length: float
    width: float
    height: Optional[float] = None
    
    def to_dict(self):
        result = {
            "length": self.length,
            "width": self.width
        }
        if self.height is not None:
            result["height"] = self.height
        return result

@dataclass
class NodePosition:
    """Node position"""
    x: float
    y: float
    map_id: str
    theta: Optional[float] = None
    allowed_deviation_xy: Optional[float] = None
    allowed_deviation_theta: Optional[float] = None
    map_description: Optional[str] = None
    
    def to_dict(self):
        result = {
            "x": self.x,
            "y": self.y,
            "mapId": self.map_id
        }
        if self.theta is not None:
            result["theta"] = self.theta
        if self.allowed_deviation_xy is not None:
            result["allowedDeviationXY"] = self.allowed_deviation_xy
        if self.allowed_deviation_theta is not None:
            result["allowedDeviationTheta"] = self.allowed_deviation_theta
        if self.map_description is not None:
            result["mapDescription"] = self.map_description
        return result

@dataclass
class Trajectory:
    """Trajectory defined as a NURBS"""
    degree: int
    knot_vector: List[float]
    control_points: List[ControlPoint]
    
    def to_dict(self):
        return {
            "degree": self.degree,
            "knotVector": self.knot_vector,
            "controlPoints": [cp.to_dict() for cp in self.control_points]
        }

@dataclass
class Velocity:
    """The AGVs velocity in vehicle coordinates"""
    vx: Optional[float] = None
    vy: Optional[float] = None
    omega: Optional[float] = None
    
    def to_dict(self):
        result = {}
        if self.vx is not None:
            result["vx"] = self.vx
        if self.vy is not None:
            result["vy"] = self.vy
        if self.omega is not None:
            result["omega"] = self.omega
        return result
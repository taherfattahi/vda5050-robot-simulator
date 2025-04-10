from dataclasses import dataclass
from typing import Dict, Optional
from protocol.vda5050_common import AgvPosition, HeaderId, Velocity

@dataclass
class Visualization:
    """AGV position and/or velocity for visualization purposes"""
    header_id: HeaderId
    timestamp: str
    version: str
    manufacturer: str
    serial_number: str
    agv_position: Optional[AgvPosition] = None
    velocity: Optional[Velocity] = None
    
    def to_dict(self) -> Dict:
        result = {
            "headerId": self.header_id,
            "timestamp": self.timestamp,
            "version": self.version,
            "manufacturer": self.manufacturer,
            "serialNumber": self.serial_number
        }
        if self.agv_position is not None:
            result["agvPosition"] = self.agv_position.to_dict()
        if self.velocity is not None:
            result["velocity"] = self.velocity.to_dict()
        return result
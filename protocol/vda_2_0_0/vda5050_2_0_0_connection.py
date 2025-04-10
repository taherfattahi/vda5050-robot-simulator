from dataclasses import dataclass
from enum import Enum
from typing import Dict
from protocol.vda5050_common import HeaderId

class ConnectionState(str, Enum):
    ONLINE = "ONLINE"  # The Connection between AGV and broker is active
    OFFLINE = "OFFLINE"  # The Connection between AGV and broker has gone offline in a coordinated way
    CONNECTION_BROKEN = "CONNECTIONBROKEN"  # The connection between AGV and broker has unexpectedly ended

@dataclass
class Connection:
    """AGV connection state reported as a last will message"""
    header_id: HeaderId
    timestamp: str
    version: str
    manufacturer: str
    serial_number: str
    connection_state: ConnectionState
    
    def to_dict(self) -> Dict:
        return {
            "headerId": self.header_id,
            "timestamp": self.timestamp,
            "version": self.version,
            "manufacturer": self.manufacturer,
            "serialNumber": self.serial_number,
            "connectionState": self.connection_state.value
        }
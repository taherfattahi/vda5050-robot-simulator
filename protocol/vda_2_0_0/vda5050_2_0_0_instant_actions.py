from dataclasses import dataclass
from typing import List, Dict
from protocol.vda5050_common import HeaderId
from protocol.vda_2_0_0.vda5050_2_0_0_action import Action

@dataclass
class InstantActions:
    """Instant actions that the AGV is to execute as soon as they arrive"""
    header_id: HeaderId
    timestamp: str
    version: str
    manufacturer: str
    serial_number: str
    instant_actions: List[Action]
    
    def to_dict(self) -> Dict:
        return {
            "headerId": self.header_id,
            "timestamp": self.timestamp,
            "version": self.version,
            "manufacturer": self.manufacturer,
            "serialNumber": self.serial_number,
            "instantActions": [action.to_dict() for action in self.instant_actions]
        }
    
    @classmethod
    def from_dict(cls, data: Dict):
        from protocol.vda_2_0_0.vda5050_2_0_0_action import Action, ActionParameter, ActionParameterValue, BlockingType
        
        instant_actions = []
        for action_data in data.get("instantActions", []):
            action_params = []
            for param in action_data.get("actionParameters", []):
                action_params.append(ActionParameter(
                    key=param["key"],
                    value=ActionParameterValue(param["value"])
                ))
            
            instant_actions.append(Action(
                action_type=action_data["actionType"],
                action_id=action_data["actionId"],
                blocking_type=BlockingType(action_data["blockingType"]),
                action_parameters=action_params,
                action_description=action_data.get("actionDescription")
            ))
            
        return cls(
            header_id=data["headerId"],
            timestamp=data["timestamp"],
            version=data["version"],
            manufacturer=data["manufacturer"],
            serial_number=data["serialNumber"],
            instant_actions=instant_actions
        )
from dataclasses import dataclass, field
from typing import List, Optional, Union
from enum import Enum, auto

class BlockingType(str, Enum):
    NONE = "NONE"  # Action can happen in parallel with others, including movement
    SOFT = "SOFT"  # Action can happen simultaneously with others, but not while moving
    HARD = "HARD"  # No other actions can be performed while this action is running

class ActionParameterValue:
    """Value of an action parameter"""
    def __init__(self, value):
        self.value = value
    
    def to_dict(self):
        return self.value

@dataclass
class ActionParameter:
    """ActionParameter Object"""
    key: str
    value: ActionParameterValue
    
    def to_dict(self):
        return {
            "key": self.key,
            "value": self.value.value
        }

@dataclass
class Action:
    """Node Action Object"""
    action_type: str
    action_id: str
    blocking_type: BlockingType
    action_parameters: List[ActionParameter]
    action_description: Optional[str] = None
    
    def to_dict(self):
        result = {
            "actionType": self.action_type,
            "actionId": self.action_id,
            "blockingType": self.blocking_type.value,
            "actionParameters": [param.to_dict() for param in self.action_parameters]
        }
        if self.action_description is not None:
            result["actionDescription"] = self.action_description
        return result
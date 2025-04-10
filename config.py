import tomli
from dataclasses import dataclass
from typing import Optional

@dataclass
class MqttBrokerConfig:
    host: str
    port: str
    vda_interface: str

@dataclass
class VehicleConfig:
    manufacturer: str
    serial_number: str
    vda_version: str
    vda_full_version: str

@dataclass
class Settings:
    action_time: float
    speed: float
    robot_count: int
    state_frequency: int
    visualization_frequency: int
    map_id: str

@dataclass
class Config:
    mqtt_broker: MqttBrokerConfig
    vehicle: VehicleConfig
    settings: Settings

def get_config() -> Config:
    """Load configuration from config.toml file"""
    with open("config.toml", "rb") as f:
        config_dict = tomli.load(f)
    
    mqtt_broker = MqttBrokerConfig(**config_dict["mqtt_broker"])
    vehicle = VehicleConfig(**config_dict["vehicle"])
    settings = Settings(**config_dict["settings"])
    
    return Config(mqtt_broker=mqtt_broker, vehicle=vehicle, settings=settings)
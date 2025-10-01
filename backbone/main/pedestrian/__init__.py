"""
Pedestrian module for CARLA scenarios
"""

from .pedestrian_controller import PedestrianController
from .pedestrian_config import get_pedestrian_model, list_available_models

__all__ = [
    'PedestrianController',
    'get_pedestrian_model',
    'list_available_models'
]



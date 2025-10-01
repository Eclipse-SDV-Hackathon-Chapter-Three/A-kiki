# Pedestrian Module

This module contains pedestrian-related functionality for CARLA scenarios.

## Files

- `pedestrian_controller.py`: Main pedestrian controller class for spawning and controlling pedestrians
- `pedestrian_config.py`: Configuration utilities for pedestrian models and settings
- `__init__.py`: Package initialization file

## Usage

```python
from pedestrian import PedestrianController, get_pedestrian_model, list_available_models

# Create a pedestrian controller
controller = PedestrianController(
    world=world,
    pedestrian_start=carla.Location(-120.0, 38.65, 1.10),
    pedestrian_end=carla.Location(-95.0, 38.65, 1.10),
    walking_speed=15.0
)

# Spawn pedestrian
controller.spawn_pedestrian()

# Update walking animation
controller.update_walking(delta_time)
```

## Features

- Automatic pedestrian spawning with fallback locations
- Configurable walking speed
- Multiple pedestrian model support
- Walking animation and direction control
- Physics effects for collision simulation

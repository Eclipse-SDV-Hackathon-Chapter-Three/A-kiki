"""
Planning module for chase vehicle
Handles path planning and decision making
"""

from .path_planner import PathPlanner
from .behavior_planner import BehaviorPlanner
from .trajectory_generator import TrajectoryGenerator

class PlanningModule:
    """Planning module wrapper"""
    
    def __init__(self):
        self.path_planner = PathPlanner()
        self.behavior_planner = BehaviorPlanner()
        self.trajectory_generator = TrajectoryGenerator()
    
    def plan_behavior(self, *args, **kwargs):
        return self.behavior_planner.plan_behavior(*args, **kwargs)
    
    def plan_path_to_target(self, *args, **kwargs):
        return self.path_planner.plan_path_to_target(*args, **kwargs)
    
    def generate_trajectory(self, *args, **kwargs):
        return self.trajectory_generator.generate_trajectory(*args, **kwargs)
    
    def cleanup(self):
        pass

__all__ = [
    'PlanningModule',
    'PathPlanner',
    'BehaviorPlanner',
    'TrajectoryGenerator'
]


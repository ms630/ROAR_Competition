"""
Competition instructions:
Please do not change anything else but fill out the to-do sections.
"""

from typing import List, Tuple, Dict, Optional
import roar_py_interface
import numpy as np

def normalize_rad(rad : float):
    return (rad + np.pi) % (2 * np.pi) - np.pi

def filter_waypoints(location : np.ndarray, current_idx: int, waypoints : List[roar_py_interface.RoarPyWaypoint]) -> int:
    def dist_to_waypoint(waypoint : roar_py_interface.RoarPyWaypoint):
        return np.linalg.norm(
            location[:2] - waypoint.location[:2]
        )
    for i in range(current_idx, len(waypoints) + current_idx):
        if dist_to_waypoint(waypoints[i%len(waypoints)]) < 3:
            return i % len(waypoints)
    return current_idx

class RoarCompetitionSolution:
    def __init__(
        self,
        maneuverable_waypoints: List[roar_py_interface.RoarPyWaypoint],
        vehicle : roar_py_interface.RoarPyActor,
        camera_sensor : roar_py_interface.RoarPyCameraSensor = None,
        location_sensor : roar_py_interface.RoarPyLocationInWorldSensor = None,
        velocity_sensor : roar_py_interface.RoarPyVelocimeterSensor = None,
        rpy_sensor : roar_py_interface.RoarPyRollPitchYawSensor = None,
        occupancy_map_sensor : roar_py_interface.RoarPyOccupancyMapSensor = None,
        collision_sensor : roar_py_interface.RoarPyCollisionSensor = None,
    ) -> None:
        self.maneuverable_waypoints = maneuverable_waypoints
        self.vehicle = vehicle
        self.camera_sensor = camera_sensor
        self.location_sensor = location_sensor
        self.velocity_sensor = velocity_sensor
        self.rpy_sensor = rpy_sensor
        self.occupancy_map_sensor = occupancy_map_sensor
        self.collision_sensor = collision_sensor
    
    async def initialize(self) -> None:
        # TODO: You can do some initial computation here if you want to.
        # For example, you can compute the path to the first waypoint.

        # Receive location, rotation and velocity data 
        vehicle_location = self.location_sensor.get_last_gym_observation()
        vehicle_rotation = self.rpy_sensor.get_last_gym_observation()
        vehicle_velocity = self.velocity_sensor.get_last_gym_observation()

        self.current_waypoint_idx = 10
        self.current_waypoint_idx = filter_waypoints(
            vehicle_location,
            self.current_waypoint_idx,
            self.maneuverable_waypoints
        )


    async def step(
        self
    ) -> None:
        """
        This function is called every world step.
        Note: You should not call receive_observation() on any sensor here, instead use get_last_observation() to get the last received observation.
        You can do whatever you want here, including apply_action() to the vehicle.
        """
        # TODO: Implement your solution here.

        # Receive location, rotation and velocity data 
        vehicle_location = self.location_sensor.get_last_gym_observation()
        vehicle_rotation = self.rpy_sensor.get_last_gym_observation()
        vehicle_velocity = self.velocity_sensor.get_last_gym_observation()
        vehicle_velocity_norm = np.linalg.norm(vehicle_velocity)
        
        # Find the waypoint closest to the vehicle
        self.current_waypoint_idx = filter_waypoints(
            vehicle_location,
            self.current_waypoint_idx,
            self.maneuverable_waypoints
        )
               

        if  (self.current_waypoint_idx < 430 or self.current_waypoint_idx > 2750):
            throttle_control = 10 * (90 - vehicle_velocity_norm)
            ah = int(0.48*vehicle_velocity_norm)
            ki = -12
        elif (self.current_waypoint_idx >= 430 and self.current_waypoint_idx < 500):
            throttle_control = 0.18 * (40 - vehicle_velocity_norm)
            ah = 16
            ki = -12
        elif (self.current_waypoint_idx >= 500 and self.current_waypoint_idx < 668):
            throttle_control = 10 * (70 - vehicle_velocity_norm)
            ah = 15
            ki = -12
        elif (self.current_waypoint_idx >= 668 and self.current_waypoint_idx < 740):
            throttle_control = 0.18 * (50 - vehicle_velocity_norm)
            ah = int(0.48*vehicle_velocity_norm)
            ki = -18
        elif (self.current_waypoint_idx >= 740 and self.current_waypoint_idx < 850):
            throttle_control = 0.18 * (52 - vehicle_velocity_norm)
            ah = int(0.48*vehicle_velocity_norm)
            ki = -18
        elif (self.current_waypoint_idx >= 850 and self.current_waypoint_idx < 890):
            throttle_control = 0.18 * (46 - vehicle_velocity_norm)
            ah = int(0.48*vehicle_velocity_norm)
            ki = -16
        elif (self.current_waypoint_idx >= 890 and self.current_waypoint_idx < 1310):
            throttle_control = 0.18 * (72 - vehicle_velocity_norm)
            ah = int(0.58*vehicle_velocity_norm)
            ki = -18
        elif (self.current_waypoint_idx >= 1310 and self.current_waypoint_idx < 1370):
            throttle_control = 0.18 * (36 - vehicle_velocity_norm)  
            ah = int(0.44*vehicle_velocity_norm)
            ki = -12
        elif (self.current_waypoint_idx >= 1370 and self.current_waypoint_idx < 1470):
            throttle_control = 0.18 * (48 - vehicle_velocity_norm)
            ah = int(0.48*vehicle_velocity_norm)
            ki = -16
        elif (self.current_waypoint_idx >= 1470 and self.current_waypoint_idx < 1870):
            throttle_control = 10 * (90 - vehicle_velocity_norm)
            ah = int(0.68*vehicle_velocity_norm)
            ki = -18
        elif (self.current_waypoint_idx >= 1870 and self.current_waypoint_idx < 1980):
            throttle_control = 0.18 * (48 - vehicle_velocity_norm)
            ah = int(0.44*vehicle_velocity_norm)
            ki = -18
        elif (self.current_waypoint_idx >= 1980 and self.current_waypoint_idx < 2600):
            throttle_control = 10 * (90 - vehicle_velocity_norm)
            ah = int(0.58*vehicle_velocity_norm)
            ki = -18
        else:
            throttle_control = 0.18 * (36 - vehicle_velocity_norm)
            ah = 16
            ki = -12



         # We use the 3rd waypoint ahead of the current waypoint as the target waypoint
        waypoint_to_follow = self.maneuverable_waypoints[(self.current_waypoint_idx + ah) % len(self.maneuverable_waypoints)]

         # We also use the 5rd waypoint ahead of the current waypoint as the correction waypoint
        waypoint_to_predict = self.maneuverable_waypoints[(self.current_waypoint_idx + 35) % len(self.maneuverable_waypoints)]

        # Calculate delta vector towards the target waypoint
        vector_to_waypoint = (waypoint_to_follow.location - vehicle_location)[:2]
        heading_to_waypoint = np.arctan2(vector_to_waypoint[1],vector_to_waypoint[0])

        vector_to_predict = (waypoint_to_predict.location - vehicle_location)[:2]
        heading_to_predict = np.arctan2(vector_to_predict[1],vector_to_predict[0])

        # Calculate delta angle towards the target waypoint
        delta_heading = normalize_rad(heading_to_waypoint - vehicle_rotation[2])

        delta_heading2 = normalize_rad(heading_to_predict - vehicle_rotation[2])
        # delta_heading = delta_heading - 0.04 * delta_heading2

        # Proportional controller to steer the vehicle towards the target waypoint
        steer_control = (
            ki/ np.sqrt(vehicle_velocity_norm) * delta_heading / np.pi
        ) if vehicle_velocity_norm > 1e-2 else -np.sign(delta_heading)
        steer_control = np.clip(steer_control, -1.0, 1.0)

        # Proportional controller to control the vehicle's speed towards 40 m/s

        


        control = {
            "throttle": np.clip(throttle_control, 0.0, 1.0),
            "steer": steer_control,
            "brake": np.clip(-throttle_control, 0.0, 1.0),
            "hand_brake": 0.0,
            "reverse": 0,
            "target_gear": 0
        }
        await self.vehicle.apply_action(control)
        return control

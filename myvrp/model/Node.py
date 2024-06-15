import math
from typing import Tuple


class Node:
    def __init__(self, id, x, y, demand):
        self.id = id
        self.x = x
        self.y = y
        self.demand = demand
        self.is_serviced = False

    def __repr__(self):
        return f"C_{self.id}"

    def distance(self, target):
        return math.sqrt(math.pow(self.x - target.x, 2) + math.pow(target.y - self.y, 2))


class RoutePoint:
    def __init__(self, node: Node, current_capacity, traveled_distance, vehicle_id):
        self.node = node
        self.current_capacity = current_capacity
        self.traveled_distance = traveled_distance
        self.vehicle_id = vehicle_id

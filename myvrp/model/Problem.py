from myvrp.model.Node import Node, RoutePoint


class Problem:
    def __init__(self, name, nodes: list[Node], distance_matrix, vehicle_capacities, num_vehicles):
        self.name = name
        self.nodes = nodes
        self.num_vehicles = num_vehicles
        self.vehicle_capacities = vehicle_capacities
        self.distance_matrix = distance_matrix
        self.depot: Node = list(filter(lambda x: x.id == 0, nodes))[0]
        self.depot.is_serviced = True

    def __repr__(self):
        return f"Instance: {self.name}\n" \
               f"Vehicle number: {self.num_vehicles}\n" \
               f"Vehicle capacity: {self.vehicle_capacities}\n"


class Route:
    def __init__(self, vehicle_id: int, nodes: list[RoutePoint], load: int, distance: int):
        self.vehicle_id = vehicle_id
        self.nodes = nodes
        self.current = nodes[0]
        self.load = load
        self.distance = distance

    def set_current(self, distance: int):
        for node in self.nodes:
            if node.traveled_distance >= distance:
                self.current = node
                break
        self.current = self.nodes[0]


class Routing:
    def __init__(self, problem: Problem, routes: list[Route]):
        self.problem = problem
        self.routes = routes

    def find_accident_routes(self, node: Node):
        for route in self.routes:
            for n in route.nodes:
                if n.node.id == node.id:
                    return route
        return None

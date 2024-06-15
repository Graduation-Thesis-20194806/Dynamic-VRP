"""Capacited Vehicles Routing Problem (CVRP)."""

import os

from myvrp.utils import load_instance
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from myvrp import BASE_DIR

from model.Node import Node, RoutePoint
from model.Problem import Problem, Route, Routing


def create_data_model(instance_name, no_nodes, customize_data=False) -> Problem:
    """Stores the data for the problem."""
    if customize_data:
        json_data_dir = os.path.join(BASE_DIR, 'data', 'json_customize')
    else:
        json_data_dir = os.path.join(BASE_DIR, 'data', 'json')
    json_file = os.path.join(json_data_dir, f'{instance_name}.json')
    instance = load_instance(json_file=json_file)
    if instance is None:
        return
    nodes = [Node(0, 0, 0, 0)]
    for i in range(1, no_nodes + 1):
        ins = instance['customer_' + str(i)]
        demand = (int(ins['demand']))
        x = int(ins['coordinates']['x'])
        y = int(ins['coordinates']['y'])
        nodes.append(Node(i, x, y, demand))
    distance = []
    for i in range(no_nodes + 1):
        distance.append([])
        for j in range(no_nodes + 1):
            distance[i].append(int(instance['distance_matrix'][i][j]))
    num_vehicles = int(instance["max_vehicle_number"])

    return Problem(instance_name, nodes, distance, [int(instance["vehicle_capacity"])] * num_vehicles, num_vehicles)


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    total_distance = 0
    total_load = 0
    for vehicle_id in range(data.num_vehicles):
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        route_distance = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load += data.nodes[node_index].demand
            plan_output += f" {node_index} Load({route_load}) -> "
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
        plan_output += f" {manager.IndexToNode(index)} Load({route_load})\n"
        plan_output += f"Distance of the route: {route_distance}m\n"
        plan_output += f"Load of the route: {route_load}\n"
        print(plan_output)
        total_distance += route_distance
        total_load += route_load
    print(f"Total distance of all routes: {total_distance}m")
    print(f"Total load of all routes: {total_load}")


def runcvrp(instance_name, no_nodes, customize_data=False):
    """Solve the CVRP problem."""
    # Instantiate the data problem.
    data = create_data_model(instance_name, no_nodes, customize_data)

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data.distance_matrix), data.num_vehicles, data.depot.id
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data.distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Capacity constraint.
    def demand_callback(from_index: int):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data.nodes[from_node].demand

    demand_callback_index = routing.RegisterUnaryTransitCallback(
        demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data.vehicle_capacities,  # vehicle maximum capacities
        True,  # start cumul to zero
        "Capacity",
    )

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.FromSeconds(1)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    return data, manager, routing, solution


def gen_map(data, manager, routing, solution) -> Routing:
    routes = []
    for vehicle_id in range(data.num_vehicles):
        nodes = []
        index = routing.Start(vehicle_id)
        route_distance = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            node = data.nodes[node_index]
            route_load += node.demand
            current_capacity = data.vehicle_capacities[vehicle_id] - route_load
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
            nodes.append(RoutePoint(node, current_capacity, route_distance, vehicle_id))
        routes.append(Route(vehicle_id, nodes, route_load, route_distance))
    return Routing(data, routes)

"""Capacited Vehicles Routing Problem (CVRP)."""

import os

from myvrp.utils import load_instance
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from myvrp import BASE_DIR
import random


def create_data_model(instance_name, no_nodes, customize_data=False):
    """Stores the data for the problem."""
    if customize_data:
        json_data_dir = os.path.join(BASE_DIR, 'data', 'json_customize')
    else:
        json_data_dir = os.path.join(BASE_DIR, 'data', 'json')
    json_file = os.path.join(json_data_dir, f'{instance_name}.json')
    instance = load_instance(json_file=json_file)
    if instance is None:
        return
    data = {}
    demand = [0]
    coordinates = [[0, 0]]
    for i in range(1, no_nodes+1):
        x = instance['customer_'+str(i)]
        demand.append(int(x['demand']))
        coordinates.append((int(x['coordinates']['x']),
                           int(x['coordinates']['y'])))
    distance = []
    for i in range(no_nodes+1):
        distance.append([])
        for j in range(no_nodes+1):
            distance[i].append(int(instance['distance_matrix'][i][j]))

    data["distance_matrix"] = distance
    data["demands"] = demand
    data["vehicle_capacities"] = [
        int(instance["vehicle_capacity"])]*instance["max_vehicle_number"]
    data["num_vehicles"] = int(instance["max_vehicle_number"])
    data["depot"] = 0
    data["coordinates"] = coordinates
    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    total_distance = 0
    total_load = 0
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        route_distance = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load += data["demands"][node_index]
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
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data["demands"][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(
        demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data["vehicle_capacities"],  # vehicle maximum capacities
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


def gen_map(data, manager, routing, solution):
    map = []
    for vehicle_id in range(data["num_vehicles"]):
        map.append([])
        index = routing.Start(vehicle_id)
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load += data['demands'][node_index]
            index = solution.Value(routing.NextVar(index))
        remain = data["vehicle_capacities"][vehicle_id] - route_load
        if (remain < 0):
            remain = 0
        index = routing.Start(vehicle_id)
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            remain += data['demands'][node_index]
            index = solution.Value(routing.NextVar(index))
            map[vehicle_id].append([node_index, remain])
        map[vehicle_id].append([0, 200])
    return map


def random_accident(map, data):
    random_integer = random.randint(1, len(data['demands'])-1)
    vehicle_id = -1
    node_id = -1
    for value in map:
        vehicle_id += 1
        node_id = -1
        check = False
        for val in value:
            node_id += 1
            if (val[0] == random_integer):
                check = True
                break
        if (check):
            break
    distance = 0
    index = 0
    r = map[vehicle_id]
    while (index != random_integer):
        if (isinstance(r[0], list) == False):
            break
        prev_index = r[0][0]
        index = r[1][0]
        distance += data['distance_matrix'][prev_index][index]
        r.pop(0)
    accident_route = []
    accident_demand = 0
    for i in range(len(r)):
        index = r[i][0]
        demand = 0
        if (i != 0):
            demand = data["demands"][index]
        accident_route.append([index, demand])
        accident_demand += demand
    map[vehicle_id] = []
    for route in map:
        dt = 0
        while dt < distance and len(route) > 1:
            if (isinstance(route[0], list) == False):
                break
            prev_index = route[0][0]
            index = route[1][0]
            dt += data['distance_matrix'][prev_index][index]
            route.pop(0)
    limit = data['distance_matrix'][0][random_integer]
    for route in map:
        i = 0
        while i < len(route):
            if (isinstance(route[0], list) == False):
                break
            index = route[i][0]
            if (data['distance_matrix'][index][random_integer] >= limit):
                route.pop(i)
            else:
                i += 1
    i = 0
    while i < len(map):
        if (len(map[i]) == 0):
            map.pop(i)
        else:
            i += 1
    return random_integer, accident_demand, accident_route

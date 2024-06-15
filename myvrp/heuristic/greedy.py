from myvrp.model.Node import Node, RoutePoint


def make_distance_sort_key(target: Node):
    def distance_sort_key(item: RoutePoint):
        return (item.node.distance(target), -item.current_capacity)

    return distance_sort_key


def arrange_available_nodes(nodes: list[RoutePoint], target: Node) -> list[RoutePoint]:
    distance_sort_key = make_distance_sort_key(target)
    return sorted(nodes, key=distance_sort_key)

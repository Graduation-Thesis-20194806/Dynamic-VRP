from myvrp.cvrp import gen_map, print_solution, random_accident, runcvrp


def main():
    data, manager, routing, solution = runcvrp('c1_2_1',200,True)
    map = gen_map(data,manager, routing, solution)
    random_integer, accident_demand, accident_route = random_accident(map,data)
    print(map)
    print(random_integer)
    print(accident_demand)
    print(accident_route)
    return


if __name__ == "__main__":
    main()

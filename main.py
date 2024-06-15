from myvrp.cvrp import gen_map, print_solution, runcvrp


def main():
    data, manager, routing, solution = runcvrp('c1_2_1',200,True)
    map = gen_map(data,manager, routing, solution)
    print_solution(data,manager,routing,solution)
    return


if __name__ == "__main__":
    main()

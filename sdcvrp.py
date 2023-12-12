import csv
from prep import *

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def create_data_model(safety_margin=0):
    data = {}
    data["distance_matrix"] = distance_matrix
    data["demands"] = demands
    data["vehicle_capacities"] = [capacity] * (num_routes + safety_margin)
    data["num_routes"] = num_routes + safety_margin
    data["depot"] = 0    
    return data

def save_solution(data, manager, routing, solution, filename):
    routes_data = []

    for route_id in range(data["num_routes"]):
        index = routing.Start(route_id)
        route_distance = 0
        route_load = 0
        route_data = []

        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            node_name = data_split[data_split['id'] == node_index]['name'].values[0]
            route_load += data["demands"][node_index]
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, route_id
            )

            route_data.append({
                "node": node_name,
                "load": data["demands"][node_index]
            })  # Add node to route data

        node_name = data_split[data_split['id'] == manager.IndexToNode(index)]['name'].values[0]
        route_data.append({
            "node": node_name,
            "load": data["demands"][manager.IndexToNode(index)]
        })

        if route_load > 0:
            routes_data.append({
                "route": route_id + 1,
                "distance": route_distance,
                "load": route_load,
                **{f"node_{i}": entry["node"] for i, entry in enumerate(route_data, 1)},
                **{f"load_{i}": entry["load"] for i, entry in enumerate(route_data, 1)}
            })

    df = pd.DataFrame(routes_data)
    df.to_csv(filename, index=False)
    return routes_data

def print_solution(data, manager, routing, solution):
    print(f"Objective: {solution.ObjectiveValue()}")
    total_distance = 0
    total_load = 0
    for route_id in range(data["num_routes"]):
        index = routing.Start(route_id)
        plan_output = f"Route {route_id + 1}:\n"
        route_distance = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load += data["demands"][node_index]
            node_name = data_split[data_split['id'] == node_index]['name'].values[0]
            plan_output += f" {node_name} Load({route_load}) -> "
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, route_id
            )
        node_name = data_split[data_split['id'] == manager.IndexToNode(index)]['name'].values[0]
        plan_output += f" {node_name} Load({route_load})\n"
        plan_output += f"Distance of the route: {float(route_distance) / 1000}km\n"
        plan_output += f"Load of the route: {route_load}\n"
        if route_load > 0:  # Check if route_load is greater than 0
            print(plan_output)
            total_distance += route_distance
            total_load += route_load
    print(f"Total distance of all routes: {float(total_distance) / 1000}km")
    print(f"Total load of all routes: {total_load}")

def custom_sort(route):
    return (route['node_2'], -route['load_2'], -route['distance'], -route['load'])

# Combine and print routes, and calculate total loads and distances
def combine_routes(routes_data):
    combined_route = []

    for route in routes_data:
        num_loads = len([key for key in route.keys() if key.startswith('load_')])
        node_sequence = [f"{route[f'node_{i}']} Load ({route[f'load_{i}']})" for i in range(1, num_loads + 1) if f'node_{i}' in route]
        combined_route += ['0 Load(0)'] + node_sequence + [f'0 Load({route[f"load_{num_loads}"]})']
    return ' -> '.join(combined_route)

def print_schedule(iteration, days, result_data):
    total_load_all = 0
    total_distance_all = 0
    total_load_all_routes = 0
    total_distance_all_routes = 0

    joined_routes_str = ''
    combined_routes = {}
    combined_total_load = 0
    combined_total_distance = 0

    for i, day_routes in enumerate(days, start=1):
        total_load_day = 0
        total_distance_day = 0
        total_load = sum(route['load'] for route in day_routes)
        total_distance = sum(route['distance'] for route in day_routes)

        total_load_all += total_load
        total_distance_all += total_distance
        total_load_all_routes += total_load
        total_distance_all_routes += total_distance 

        routes = [f'Route {row["route"]}' for row in day_routes]
        routes_str = ' -> '.join(routes)
        
        vehicle = (i - 1) % num_vehicles + 1
        adjusted_day_number = (i - 1) % 7 + 1

        if i == 1 or (i - 1) % 7 == 0:
            vehicle = (i - 1) // 7 + 1
            print(f"Vehicle {vehicle}:")

        vehicle_data = {
            'day_id': adjusted_day_number,
            'total_distance': total_distance,
            'total_load': total_load,
            'routes': routes_str
        }
        result_data.append(vehicle_data)

        if adjusted_day_number in (1, 2):
            # Combine routes for day 1 and 2
            if adjusted_day_number not in combined_routes:
                combined_routes[adjusted_day_number] = []
            combined_routes[adjusted_day_number] += day_routes
            combined_total_load += total_load
            combined_total_distance += total_distance
        else:
            print(f"Day {adjusted_day_number}:")
            print(routes_str)
            print()

            joined_routes_str += routes_str
            if (i % 7) != 0 and i != len(days):
                joined_routes_str += ' -> '

            for j, route in enumerate(day_routes, start=1):

                route_sequence = f"Route {route['route']}: "
                route_load = 0
                route_distance = 0

                num_nodes = len([key for key in route.keys() if key.startswith('node_')])
                cumulative_load = 0
                for j in range(1, num_nodes + 1):
                    node = route[f'node_{j}']
                    load = route[f'load_{j}']
                    cumulative_load += load
                    route_sequence += f"{node} ({cumulative_load})kg"
                    if j < num_nodes:
                        route_sequence += " -> "
                    else:
                        route_sequence += "\n"
                print(route_sequence, end="")
            print(f"\nTotal Loads: {total_load}kg")
            print(f"Total Distances: {float(total_distance) / 1000}km\n")

            if i % 7 == 0 or i == len(days):
                joined_routes_str += '\n'
                vehicle = (i - 1) // 7 + 1
                print(f"Vehicle {vehicle}: " + joined_routes_str)
                print(f"Total Loads: {total_load_all}kg")
                print(f"Total Distances: {float(total_distance_all) / 1000}km\n")
                joined_routes_str = ''
                total_load_all = 0
                total_distance_all = 0

        if adjusted_day_number == 2:
            # Print combined routes for days 1 and 2 before printing day 3
            combined_routes_str = ' -> '.join([f'Route {row["route"]}' for row in combined_routes[1]]) + ' -> ' + ' -> '.join([f'Route {row["route"]}' for row in combined_routes[2]])
            print(f"Day 1 & 2:")
            print(combined_routes_str)
            print()

            for day_number in [1, 2]:
                vehicle_routes = combined_routes[day_number]
                total_load_day = sum(route['load'] for route in vehicle_routes)
                total_distance_day = sum(route['distance'] for route in vehicle_routes)

                for route in vehicle_routes:
                    num_nodes = len([key for key in route.keys() if key.startswith('node_')])
                    cumulative_load = 0
                    route_sequence = f"Route {route['route']}: "
                    for j in range(1, num_nodes + 1):
                        node = route[f'node_{j}']
                        load = route[f'load_{j}']
                        cumulative_load += load
                        route_sequence += f"{node} ({cumulative_load})kg"
                        if j < num_nodes:
                            route_sequence += " -> "
                        else:
                            route_sequence += "\n"
                    print(route_sequence, end="")
            print(f"\nTotal Loads: {combined_total_load}kg")
            print(f"Total Distances: {float(combined_total_distance) / 1000}km\n")

            combined_routes = {}
            combined_total_load = 0
            combined_total_distance = 0
    print(f"Total Loads: {total_load_all_routes}kg\nTotal Distances: {float(total_distance_all_routes) / 1000}km")
    # print(result_data)

def save_schedule(iteration, days, result_data, save_path):
    csv_file_path = save_path
    schedule_data = {}

    # Iterate through the result_data and organize it into a schedule format
    for vehicle_data in result_data:
        day_id = vehicle_data['day_id']
        routes = vehicle_data['routes']
        combined_routes = '\n'.join(routes.split(' -> '))
        if day_id not in schedule_data:
            schedule_data[day_id] = {}
        schedule_data[day_id][f'Vehicle {len(schedule_data[day_id]) + 1}'] = combined_routes

    # Write the schedule_data to a CSV file
    with open(csv_file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        
        # Write the header row
        writer.writerow(['', *[f'Day {i}' for i in range(1, len(schedule_data) + 1)]])
        
        # Iterate through the days
        for day_number in range(1, max(len(v) for v in schedule_data.values()) + 1):
            row = [f'Vehicle {day_number}']
            for vehicle_id in schedule_data.keys():
                route = schedule_data[vehicle_id].get(f'Vehicle {day_number}', '')
                row.append(route)
            writer.writerow(row)

    print(f'Schedule saved to {csv_file_path}.')

def main(iteration):
    # Instantiate the data problem.
    data = create_data_model()
    solutions = []
    objective_values = []
    safety_margin = 0
    solution_found = False
    while True:
        data = create_data_model(safety_margin)

        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(
            len(data["distance_matrix"]), data["num_routes"], data["depot"]
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

        demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
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
            routing_enums_pb2.FirstSolutionStrategy.LOCAL_CHEAPEST_ARC
        )
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GENERIC_TABU_SEARCH
        )
        search_parameters.time_limit.FromSeconds(60)

        # Solve the problem.
        solution = routing.SolveWithParameters(search_parameters)
        
        # Print on console
        if solution:
            print_solution(data, manager, routing, solution)
            for i in range(1, 61):
                search_parameters2 = pywrapcp.DefaultRoutingSearchParameters()
                search_parameters2.first_solution_strategy = (   
                    routing_enums_pb2.FirstSolutionStrategy.LOCAL_CHEAPEST_ARC
                )
                search_parameters2.local_search_metaheuristic = (
                    routing_enums_pb2.LocalSearchMetaheuristic.GENERIC_TABU_SEARCH
                )
                search_parameters2.time_limit.FromSeconds(i)
                new_solution = routing.SolveWithParameters(search_parameters2)
                if new_solution:
                    print_solution(data, manager, routing, new_solution)
                    file_name = f"sdcvrp/LOCAL_CHEAPEST_ARC/GENERIC_TABU_SEARCH/result_{iteration}_solution_{i}.csv"
                    save_path = f"sdcvrp/LOCAL_CHEAPEST_ARC/GENERIC_TABU_SEARCH/schedule_{iteration}_solution_{i}.csv"
                    objective_value = new_solution.ObjectiveValue()
                    routes_data = save_solution(data, manager, routing, new_solution, file_name)

                    total_distance = sum(route['distance'] for route in routes_data)
                    safety_margin2 = 0
                    match_found = False

                    while not match_found:
                        distance_limit_per_day = total_distance / (num_vehicles*7) + safety_margin2
                        days = [[] for _ in range(num_vehicles*7)]

                        current_day_index = 0
                        current_day_distance = 0

                        for route in routes_data:
                            if current_day_distance + route['distance'] <= distance_limit_per_day:
                                days[current_day_index].append(route)
                                current_day_distance += route['distance']
                            else:
                                current_day_index += 1
                                current_day_distance = 0
                                if current_day_index >= num_vehicles*7:
                                    break
                                days[current_day_index].append(route)
                                current_day_distance += route['distance']
                        total_distance_all = sum(route['distance'] for day_routes in days for route in day_routes)

                        if total_distance_all == total_distance:
                            match_found = True
                        else:
                            safety_margin2 += 1
                    result_data = []
                    print_schedule(i, days, result_data)
                    save_schedule(i, days, result_data, save_path)

                    solutions.append((routes_data, objective_value))
                    objective_values.append((i, objective_value))

                    with open(f"sdcvrp/LOCAL_CHEAPEST_ARC/GENERIC_TABU_SEARCH/sdcvrp_objectives_{iteration}.csv", mode='w', newline='') as file:
                        writer = csv.writer(file)
                        writer.writerow(['parameter', 'objective_value'])
                        writer.writerows(objective_values)
            solution_found = True
            return solutions
        else:
            print(f"No solution {num_routes + safety_margin}.")
            safety_margin += 1

if __name__ == "__main__":
    for i in range(1, 12):  # Run 11
        main(i)

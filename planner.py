from __future__ import print_function
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
import requests, config

###########################
# Problem Data Definition #
###########################
def create_data_model():
  """Stores the data for the problem"""
  data = {}

  # create the locations and demands
  locations = open('locations.txt', 'r').read().splitlines()
  demands = open('weights.txt', 'r').read().splitlines()
  length = len(demands)
  for i in range(0, length):
      demands[i] = int(demands[i])

  # creates the parts of the url for the get request
  key = '&key=' + config.apiKey

  baseURL = 'https://maps.googleapis.com/maps/api/distancematrix/json?'

  origin = 'origins='

  destinations = '&destinations='
  
  for location in locations:
      destinations += location.replace(' ', '') + '|'

  _distances = []

  tempList = []

  for lcoation in locations:
      request = requests.get(baseURL + origin + location.replace(' ', '') + destinations + key).json()
      request = request['rows'][0]['elements']
      for elem in request:
          print(elem)
          tempList.append(int(elem['duration']['value']))
      _distances.append(tempList)
      tempList = []
  
  capacities = [700]
  data["distances"] = _distances
  data["num_locations"] = len(_distances)
  data["num_vehicles"] = 1
  data["depot"] = 0
  data["demands"] = demands
  data["vehicle_capacities"] = capacities
  return data

#######################
# Problem Constraints #
#######################
#def manhattan_distance(position_1, position_2):
#  """Computes the Manhattan distance between two points"""
#  return (abs(position_1[0] - position_2[0]) + abs(position_1[1] - position_2[1]))

def create_distance_callback(data):
  """Creates callback to return distance between points."""
  distances = data["distances"]

  def distance_callback(from_node, to_node):
    """Returns the manhattan distance between the two nodes"""
    return distances[from_node][to_node]
  return distance_callback

def create_demand_callback(data):
    """Creates callback to get demands at each location."""
    def demand_callback(from_node, to_node):
        return data["demands"][from_node]
    return demand_callback

def add_capacity_constraints(routing, data, demand_callback):
    """Adds capacity constraint"""
    capacity = "Capacity"
    routing.AddDimensionWithVehicleCapacity(
        demand_callback,
        0, # null capacity slack
        data["vehicle_capacities"], # vehicle maximum capacities
        True, # start cumul to zero
        capacity)
    
def add_distance_dimension(routing, distance_callback):
    distance = 'Distance'
    maximum_distance = 648000
    routing.AddDimension(distance_callback, 0, maximum_distance, True, distance)
    distance_dimension = routing.GetDimensionOrDie(distance)
    #distance_dimension.SetGlobalSpanCostCoefficient(100)

###########
# Printer #
###########
def print_solution(data, routing, assignment):
    """Print routes on console."""
    total_dist = 0
    visited_locations = []
    dropped_locations = list(range(data["num_locations"]))

    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {0}:\n'.format(vehicle_id)
        route_dist = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = routing.IndexToNode(index)
            next_node_index = routing.IndexToNode(assignment.Value(routing.NextVar(index)))
            visited_locations.append(next_node_index)
            if next_node_index in dropped_locations:
                dropped_locations.remove(next_node_index)
            route_dist += routing.GetArcCostForVehicle(node_index, next_node_index, vehicle_id)
            route_load += data["demands"][node_index]
            plan_output += ' {0} Load({1}) -> '.format(node_index, route_load)
            index = assignment.Value(routing.NextVar(index))

        node_index = routing.IndexToNode(index)
        total_dist += route_dist
        plan_output += ' {0} Load({1})\n'.format(node_index, route_load)
        plan_output += 'Distance of the route: {0}m\n'.format(route_dist)
        plan_output += 'Load of the route: {0}\n'.format(route_load)
        print(plan_output)
    print('Total Distance of all routes: {0}m'.format(total_dist))
    print('Dropped visits: ', list(set(range(data["num_locations"])) - set(visited_locations)))

########
# Main #
########
def main():
  """Entry point of the program"""
  # Instantiate the data problem.
  data = create_data_model()
  # Create Routing Model
  routing = pywrapcp.RoutingModel(
      data["num_locations"],
      data["num_vehicles"],
      data["depot"])
  # Define weight of each edge
  distance_callback = create_distance_callback(data)
  add_distance_dimension(routing, distance_callback)
  routing.SetArcCostEvaluatorOfAllVehicles(distance_callback)
  # Add Capacity constraint
  demand_callback = create_demand_callback(data)
  add_capacity_constraints(routing, data, demand_callback)
  # Setting first solution heuristic (cheapest addition).
  search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
  search_parameters.first_solution_strategy = (
      routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

  # Adding penalty costs to allow dropping visits.
  penalty = 1000000
  for i in range(1, data["num_locations"]):
    routing.AddDisjunction([routing.NodeToIndex(i)], penalty)
  # Solve the problem.
  assignment = routing.SolveWithParameters(search_parameters)
  if assignment:
    print_solution(data, routing, assignment)

if __name__ == '__main__':
    main()

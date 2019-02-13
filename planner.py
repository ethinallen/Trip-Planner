from __future__ import print_function
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
import requests, config

###########################
# Problem Data Definition #
###########################
def create_data_model():
  # stores all of the data for the problem
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

  for location in locations:
      request = requests.get(baseURL + origin + location.replace(' ', '') + destinations + key).json()
      request = request['rows'][0]['elements']
      for elem in request:
        try:
          tempList.append(int(elem['duration']['value']))
        except:
          tempList.append(100000)
      _distances.append(tempList)
      tempList = []
  
  # the capacity of the vehicle; this is a random number because we are ideally not listing locations that we do not want
  capacities = [400]
  data["times"] = _distances
  data["num_locations"] = len(_distances)
  data["num_vehicles"] = 1
  data["depot"] = 0
  data["demands"] = demands
  data["vehicle_capacities"] = capacities
  data['locations'] = locations
  return data

#######################
# Problem Constraints #
#######################i

# google used to have a 'manhattan distance' function here and if the program quits working it is quite likely becaue of this

# returns the distances between all points (list of lists)
def create_time_callback(data):
  """Creates callback to return distance between points."""
  times = data["times"]

# returns the distance between two nodes
  def time_callback(from_node, to_node):
    """Returns the manhattan distance between the two nodes"""
    return times[from_node][to_node]
  return time_callback

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
    
def add_time_dimension(routing, time_callback):
    time = 'Distance'
    maximum_time = 648000
    routing.AddDimension(time_callback, 0, maximum_time, True, time)
    time_dimension = routing.GetDimensionOrDie(time)
    #distance_dimension.SetGlobalSpanCostCoefficient(100)

###########
# Printer #
###########
def print_solution(data, routing, assignment):
    """Print routes on console."""
    total_time = 0
    visited_locations = []
    dropped_locations = list(range(data["num_locations"]))

    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {0}:\n'.format(vehicle_id)
        route_time = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = routing.IndexToNode(index)
            next_node_index = routing.IndexToNode(assignment.Value(routing.NextVar(index)))
            visited_locations.append(next_node_index)
            if next_node_index in dropped_locations:
                dropped_locations.remove(next_node_index)
            route_time += routing.GetArcCostForVehicle(node_index, next_node_index, vehicle_id)
            route_load += data["demands"][node_index]
            plan_output += ' {0} -> '.format(data['locations'][node_index])
            index = assignment.Value(routing.NextVar(index))

        node_index = routing.IndexToNode(index)
        total_time += route_time
        plan_output += ' {0} Load({1})\n'.format(data['locations'][node_index], route_load)
        plan_output += 'Time of the route: {0}m\n'.format(route_time)
        print(plan_output)
    print('Total Time of all routes: {0}m'.format(total_time))
    droppedLocations = list(set(range(data["num_locations"])) - set(visited_locations))
    droppedNames = []
    for location in droppedLocations:
        droppedNames.append(data['locations'][location])
    print('Dropped visits: ', droppedNames)

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
  time_callback = create_time_callback(data)
  add_time_dimension(routing, time_callback)
  routing.SetArcCostEvaluatorOfAllVehicles(time_callback)
  # Add Capacity constraint
  demand_callback = create_demand_callback(data)
  add_capacity_constraints(routing, data, demand_callback)
  # Setting first solution heuristic (cheapest addition).
  search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
  search_parameters.first_solution_strategy = (
      routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

  # Adding penalty costs to allow dropping visits.
  penalty = 4000000
  for i in range(1, data["num_locations"]):
    routing.AddDisjunction([routing.NodeToIndex(i)], penalty)
  # Solve the problem.
  assignment = routing.SolveWithParameters(search_parameters)
  if assignment:
    print_solution(data, routing, assignment)

if __name__ == '__main__':
    main()

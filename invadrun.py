import numpy as np
import re
import json
import osmnx as ox
import networkx as nx
import gpxpy
import gpxpy.gpx
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import requests
from concurrent.futures import ThreadPoolExecutor, as_completed


def save_matrix_to_npy(matrix, filename):
    """Saves a 2D matrix to a NumPy .npy file."""
    np.save(filename, matrix)
    print(f"Matrix saved to {filename}.npy")


def load_matrix_from_npy(filename):
    """Loads a 2D matrix from a NumPy .npy file."""
    matrix = np.load(filename)
    print(f"Matrix loaded from {filename}")
    return matrix

# Function to get elevation data from OpenElevation API


def get_elevation(lat, lon):
    url = f"https://api.open-elevation.com/api/v1/lookup?locations={lat},{lon}"
    response = requests.get(url).json()
    return response['results'][0]['elevation']

# Function to compute the shortest path length between two points using OSMNX


def compute_shortest_path(G, coord1, coord2):
    try:
        node1 = ox.nearest_nodes(G, coord1[1], coord1[0])
        node2 = ox.nearest_nodes(G, coord2[1], coord2[0])
        return nx.shortest_path_length(G, node1, node2, weight='length')
    except Exception as e:
        print(f"Error computing path between {coord1} and {coord2}: {e}")
        return float('inf')


def compute_distance_matrix_multithreaded(G, coordinates, max_workers=32):
    size = len(coordinates)
    distance_matrix = [[float('inf')] * size for _ in range(size)]

    # Using ThreadPoolExecutor for multi-threading
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        future_to_points = {
            executor.submit(compute_shortest_path, G, coordinates[i], coordinates[j]): (i, j)
            for i in range(size) for j in range(i + 1, size)
        }

        # As threads complete, assign the calculated values to the matrix
        for future in as_completed(future_to_points):
            i, j = future_to_points[future]
            try:
                distance = future.result()
                print(i, j)
                distance_matrix[i][j] = distance
                distance_matrix[j][i] = distance  # Symmetric matrix
            except Exception as exc:
                print(f"Error occurred for points ({i}, {j}): {exc}")

    return distance_matrix

# Function to solve TSP with OR-Tools, using the street-based distance matrix


def solve_tsp_with_streets(distance_matrix, depot=0):
    tsp_size = len(distance_matrix)
    num_routes = 1

    manager = pywrapcp.RoutingIndexManager(tsp_size, num_routes, depot)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    solution = routing.SolveWithParameters(search_parameters)
    if not solution:
        return None

    index = routing.Start(0)
    plan = []
    while not routing.IsEnd(index):
        plan.append(manager.IndexToNode(index))
        index = solution.Value(routing.NextVar(index))
    return plan

# Function to create GPX file with waypoints, route points, and track points


def create_gpx_with_streets(coordinates, path, G):
    gpx = gpxpy.gpx.GPX()

    # Add track
    gpx_track = gpxpy.gpx.GPXTrack()
    gpx.tracks.append(gpx_track)
    gpx_segment = gpxpy.gpx.GPXTrackSegment()
    gpx_track.segments.append(gpx_segment)

    # Follow the TSP path and extract the full street route
    for i in range(len(path) - 1):
        start_coord = coordinates[path[i]]
        end_coord = coordinates[path[i + 1]]

        # Waypoint (for important GPS points)
        waypoint = gpxpy.gpx.GPXWaypoint(
            latitude=start_coord[0], longitude=start_coord[1], name=start_coord[2])
        gpx.waypoints.append(waypoint)

        # Get street nodes for the start and end coordinates
        start_node = ox.nearest_nodes(G, start_coord[1], start_coord[0])
        end_node = ox.nearest_nodes(G, end_coord[1], end_coord[0])

        # Get the shortest path as street nodes
        street_path = nx.shortest_path(
            G, start_node, end_node, weight='length')

        for node in street_path:
            lat = G.nodes[node]['y']
            lon = G.nodes[node]['x']
            # elevation = get_elevation(lat, lon)

            # Track Point
            track_point = gpxpy.gpx.GPXTrackPoint(
                latitude=lat, longitude=lon)
            gpx_segment.points.append(track_point)

    return gpx


# Example: Replace with your GPS coordinates
coordinates = []
with open('invaders.json') as i:
    coordinates = list(map(lambda f: (f['geometry']['coordinates'][1], f['geometry']['coordinates'][0], f['properties']['name']), filter(
        lambda f: re.match(r".*PA_.*", f['properties']['name']), json.load(i)['features'])))


coordinates = coordinates[0:100]

# Step 1: Download the street network for the area that covers all coordinates
G = None
try:
    print('load graph from file')
    G = ox.load_graphml('paris.graphml')
except:
    print('unable to load graph. starting a new one.')
    ox.config(use_cache=True, log_console=True)
    G = ox.graph_from_place('Paris, France', network_type='walk')
    ox.io.save_graphml(G, 'paris.graphml')

# Step 2: Create distance matrix using street network
distance_matrix = None
try:
    print('load distance matrix.')
    distance_matrix = load_matrix_from_npy("invaders_matrix.npy")
except:
    print('fail to load distance matrix, starting a new one.')
    distance_matrix = compute_distance_matrix_multithreaded(G, coordinates)
    save_matrix_to_npy(distance_matrix, "invaders_matrix")

# Step 3: Solve TSP for the street-based distance matrix
start_point = list(map(
    lambda c: c[2] == 'Space Invader PA_0041, Porte de la Villette', coordinates)
).index(True)
print('using start point', start_point)
tsp_path = solve_tsp_with_streets(distance_matrix, start_point)

# Step 4: Create GPX file with waypoints, route points, and track points
if tsp_path:
    gpx = create_gpx_with_streets(coordinates, tsp_path, G)
    with open("route_with_street_map.gpx", "w") as f:
        f.write(gpx.to_xml())
    print("GPX file 'route_with_street_map.gpx' created successfully!")
else:
    print("No solution found!")

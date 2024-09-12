import osmnx as ox
import networkx as nx
import heapq
import json

def dijkstra_algorithm(graph, start_node, end_node):
    distances = {node: float('inf') for node in graph.nodes}
    distances[start_node] = 0
    paths = {node: [] for node in graph.nodes}
    paths[start_node] = [start_node]
    priority_queue = [(0, start_node)]
    explored_nodes = []
    edges_explored = []

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        if current_distance > distances[current_node]:
            continue

        if current_node not in explored_nodes:
            explored_nodes.append(current_node)

        for neighbor in graph.neighbors(current_node):
            weight = graph[current_node][neighbor].get('weight', 1)
            distance = current_distance + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                paths[neighbor] = paths[current_node] + [neighbor]
                heapq.heappush(priority_queue, (distance, neighbor))
                edges_explored.append((current_node, neighbor))

                if neighbor == end_node:
                    return paths[neighbor], distances[end_node], explored_nodes, edges_explored

    return [], float('inf'), explored_nodes, edges_explored

place_name = "Kathmandu Valley, Nepal"

#  Specify start and target points (use lat/lon of your choice)
start_latlng = (27.686154, 85.315757)  
end_latlng = (27.682096, 85.319508)    

#Get the graph for Kathmandu
G = ox.graph_from_place(place_name, network_type='drive')
gdf_nodes, gdf_edges = ox.graph_to_gdfs(G, nodes=True, edges=True)

# Add 'weight' for edges based on 'length' for Dijkstra
for u, v, data in G.edges(data=True):
    data['weight'] = data.get('length', 1)

if len(G.nodes) < 2:
    raise ValueError("Graph must have at least two nodes to compute the shortest path.")

# Finding nearest nodes to the start and end lat/lon
source_node = ox.distance.nearest_nodes(G, start_latlng[1], start_latlng[0])
target_node = ox.distance.nearest_nodes(G, end_latlng[1], end_latlng[0])

#  Runin Dijkstra algorithm to find the shortest path
shortest_path, path_length, explored_nodes, edges_explored = dijkstra_algorithm(G, source_node, target_node)

#  Prepare data for export
explored_nodes_coords = [[gdf_nodes.loc[node, 'y'], gdf_nodes.loc[node, 'x']] for node in explored_nodes]
edges_explored_coords = [[[gdf_nodes.loc[u, 'y'], gdf_nodes.loc[u, 'x']], [gdf_nodes.loc[v, 'y'], gdf_nodes.loc[v, 'x']]] for u, v in edges_explored]
shortest_path_coords = [(gdf_nodes.loc[node, 'y'], gdf_nodes.loc[node, 'x']) for node in shortest_path]

data = {
    "explored_nodes": explored_nodes_coords,
    "edges_explored": edges_explored_coords,
    "shortest_path": shortest_path_coords,
    "start_node": [gdf_nodes.loc[source_node, 'y'], gdf_nodes.loc[source_node, 'x']],
    "end_node": [gdf_nodes.loc[target_node, 'y'], gdf_nodes.loc[target_node, 'x']]
}

#  data to a JSON file for visualization
with open('dijkstras_data.json', 'w') as f:
    json.dump(data, f)

print("Shortest path length:", path_length) 
print("JSON data saved as 'dijkstras_data.json'.")

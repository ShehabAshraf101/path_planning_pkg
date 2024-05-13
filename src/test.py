def count_edges_per_point(edges):
    edge_count = {}
    for edge in edges:
        for point in edge:
            if point in edge_count:
                edge_count[point] += 1
            else:
                edge_count[point] = 1
    return edge_count

# List of edges
unique_edges = {((21.9125, -25.725), (21.9125, 0)), ((0, 25.7625), (21.9125, 25.7625)), ((-22.05, -25.725), (-22.05, 0)), ((-22.05, 25.7625), (0, 25.7625)), ((-22.05, 0), (0, 0)), ((0, -25.725), (21.9125, -25.725)), ((0, 0), (0, 25.7625)), ((-22.05, 0), (-22.05, 25.7625)), ((21.9125, 0), (21.9125, 25.7625)), ((0, 0), (21.9125, 0)), ((0, -25.725), (0, 0)), ((-22.05, -25.725), (0, -25.725))}

edge_count_per_point = count_edges_per_point(unique_edges)
# Filter edges containing the point (0, -25.725)
edges_containing_point = {edge for edge in unique_edges if (0, -25.725) in edge}

# Print edges containing the point (0, -25.725)
for edge in edges_containing_point:
    print(edge)

# Print the number of edges for each point
for point, count in edge_count_per_point.items():
    print(f"Point {point}: {count} edges")

# # Define the A* algorithm
# def astar(start, goal):
#     open_set = {start}
#     came_from = {}
#     g_score = {point: float('inf') for point in points}
#     g_score[start] = 0
#     f_score = {point: float('inf') for point in points}
#     f_score[start] = manhattan_distance(start, goal)

#     while open_set:
#         current = min(open_set, key=lambda point: f_score[point])
#         if current == goal:
#             path = [current]
#             while current in came_from:
#                 current = came_from[current]
#                 path.append(current)
#             return path[::-1]

#         open_set.remove(current)
#         for neighbor in get_neighbors(current,get_edges(points)):
#             tentative_g_score = g_score[current] + 1  # Assuming uniform cost for each step
#             if tentative_g_score < g_score[neighbor]:
#                 came_from[neighbor] = current
#                 g_score[neighbor] = tentative_g_score
#                 f_score[neighbor] = g_score[neighbor] + manhattan_distance(neighbor, goal)
#                 if neighbor not in open_set:
#                     open_set.add(neighbor)

#     return None  # No path found
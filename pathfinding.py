import heapq

# utility function to find the distance between two nodes
def distance(node1, node2):
  return ((node1[0] - node2[0]) ** 2) + ((node1[1] - node2[1]) ** 2)

# utility function to check if a given node is within the boundaries of the grid
def is_valid_node(node, grid):
  return 0 <= node[0] < len(grid) and 0 <= node[1] < len(grid[0])

# utility function to retrieve the neighbors of a given node
def get_neighbors(node, grid):
  neighbors = []
  x, y = node
  for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
    new_x, new_y = x + dx, y + dy
    if is_valid_node((new_x, new_y), grid) and grid[new_x][new_y] != -1:
      neighbors.append((new_x, new_y))
  return neighbors

# utility function to get the cost of moving from one node to another
def get_cost(node1, node2, grid):
  if node2 in get_neighbors(node1, grid):
    return distance(node1, node2)
  return float('inf')

# utility function to reconstruct the path from the came_from map
def reconstruct_path(came_from, start, goal):
  path = [goal]
  current = goal
  while current != start:
    current = came_from[current]
    path.append(current)
  return path[::-1]

# function to find the shortest path between two nodes using A* search
def a_star_search(start, goal, grid):
  # initialize the came_from map and the g_score and f_score maps
  came_from = {}
  g_score = {start: 0}
  f_score = {start: distance(start, goal)}

  # use a heap to efficiently find the lowest f_score node
  heap = [(f_score[start], start)]
  while heap:
    current = heapq.heappop(heap)[1]
    if current == goal:
      return reconstruct_path(came_from, start, goal)

    for neighbor in get_neighbors(current, grid):
      tentative_g_score = g_score[current] + get_cost(current, neighbor, grid)
      if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
        came_from[neighbor] = current
        g_score[neighbor] = tentative_g_score
        f_score[neighbor] = g_score[neighbor] + distance(neighbor, goal)
        heapq.heappush(heap, (f_score[neighbor], neighbor))
  return []

# grid
grid = [[0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0]]

start_x, start_y = map(int, input('Enter start node coordinates (x y): ').split())
start = (start_x, start_y)

goal_x, goal_y = map(int, input('Enter goal node coordinates (x y): ').split())
goal = (goal_x, goal_y)

print('Enter blocked node coordinates (x y) or enter nothing to continue:')
while True:
  coordinates = input()
  if not coordinates:
    break
  x, y = map(int, coordinates.split())
  grid[x][y] = -1

path = a_star_search(start, goal, grid)
print(path)
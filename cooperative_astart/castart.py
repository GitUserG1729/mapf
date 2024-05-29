def euclidean_distance(node, goal):
    """
    Calculates the Euclidean distance between two nodes.

    :param node: Starting node.
    :param goal: Goal node.
    :return: Euclidean distance between the two nodes.
    """
    return ((node[0] - goal[0])**2 + (node[1] - goal[1])**2)**0.5


def manhattan_distance(node, goal):
    """
    Calculates the Manhattan distance between two nodes.

    :param node: Starting node.
    :param goal: Goal node.
    :return: Manhattan distance between the two nodes.
    """
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

def true_distance(node, goal, map):
    
    return 


def valid(node, map, reserved_table, obstacles):
    valid = True 
    x, y, t = node[0], node[1], node[2]

    if node in reserved_table:
        valid = False 
    
    if (x, y) in obstacles:
        valid = False 
    
    if not ((0 <= x and x < map[0]) and (0 <= y and y < map[1])):
        valid = False

    return valid


def neighbors_func(current, map, reserved_table, obstacles): 
    neighbors = [] 
    directions = [[0, 1], [0, -1], [1, 0], [-1, 0]]

    # up, down, left, right 
    for delta in directions:
        x_n, y_n = current[0] + delta[0], current[1] + delta[1] 
        t = current[2]
        if valid((x_n, y_n, t+1), map, reserved_table, obstacles):
            neighbors.append((x_n, y_n, t+1))

    # choose puse
    if (current[0], current[1], t+1) not in reserved_table and [current[0], current[0]] not in obstacles:
        neighbors.append((current[0], current[1], t+1))

    return neighbors


def astar3d(start, goal, h_func, neighbors_func, map, obstacles, reserved_table):
    """
    A* algorithm implementation.

    :param start: Starting node.
    :param goal: Goal node.
    :param h_func: Heuristic function.
    :param neighbors_func: Function to get neighbors of a node.
    :return: Path from start to goal.
    """
    # Initialize open and closed sets
    open_set = {start}
    closed_set = set()

    # Initialize g and f values
    g = {start: 0}
    f = {start: h_func(start, goal)}

    # Initialize came_from dictionary
    came_from = {}

    while open_set:
        # Get node with lowest f value
        current = min(open_set, key=lambda x: f[x])

        # Check if current node is goal
        if [current[0], current[1]] == [goal[0], goal[1]]:
            # Reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            print(open_set)
            return path

        # Move current node from open to closed set
        open_set.remove(current)
        closed_set.add(current)

        # Get neighbors of current node
        neighbors = neighbors_func(current, map, reserved_table, obstacles)

        for neighbor in neighbors:
            # Check if neighbor is already in closed set
            if neighbor in closed_set:
                continue

            # Calculate tentative g score
            tentative_g = g[current] + 1

            # Check if neighbor is not in open set or if tentative g score is lower than previous g score
            if neighbor not in open_set or tentative_g < g[neighbor]:
                # Update came_from, g, and f values
                came_from[neighbor] = current
                g[neighbor] = tentative_g
                f[neighbor] = g[neighbor] + h_func(neighbor, goal)

                # Add neighbor to open set if it's not already there
                if neighbor not in open_set:
                    open_set.add(neighbor)

    print(closed_set)

    # If goal is not found, return None
    return None


def update_reserved_table(path, reserved_table):
    for node in path:
        reserved_table.add(node) 
    return reserved_table







def main():

    map = [6, 6]

    # map 6 x 6
    obstacles = [
        (1, 0),
        (1, 4),
        (2, 3),
        (3, 3),
        (3, 5), 
        (4, 0),
        (4, 1), 
        (4, 2), 
        (5, 4)
    ]

    start = [0, 0]
    goal = [5, 0] 
    reserved_table = {} 

    start = (0, 0, 0)
    ans = astar3d(start, goal, manhattan_distance, neighbors_func, map, obstacles, reserved_table)
    print("ans = ", ans)


main()



# f =  {(0, 0, 0): 5, (0, 1, 1): 7, (0, 0, 1): 6, (0, 1, 2): 8, (0, 0, 2): 7, (0, 1, 3): 9, (0, 0, 3): 8, (0, 2, 2): 9, (1, 1, 2): 7, (1, 2, 3): 9, (2, 1, 3): 7, (1, 1, 3): 8, (2, 2, 4): 9, (2, 0, 4): 7, (3, 1, 4): 7, (1, 1, 4): 9, (2, 1, 4): 8, (3, 2, 5): 9, (3, 0, 5): 7, (2, 1, 5): 9, (3, 1, 5): 8, (2, 0, 5): 8, (3, 1, 6): 9, (2, 0, 6): 9, (3, 0, 6): 8, (2, 1, 6): 10, (3, 2, 6): 10, (1, 2, 4): 10, (0, 1, 4): 10, (3, 1, 7): 10, (2, 0, 7): 10, (3, 0, 7): 9, (0, 2, 3): 10, (0, 0, 4): 9, (2, 2, 5): 10, (1, 1, 5): 10, (3, 1, 8): 11, (2, 0, 8): 11, (3, 0, 8): 10, (0, 2, 4): 11, (0, 1, 5): 11, (0, 0, 5): 10, (0, 3, 3): 11, (1, 2, 5): 11, (2, 2, 6): 11, (1, 1, 6): 11, (2, 1, 7): 11, (1, 3, 4): 11, (3, 2, 7): 11, (2, 2, 7): 12, (1, 1, 7): 12, (1, 2, 6): 12, (2, 1, 8): 12, (1, 3, 5): 12, (0, 2, 5): 12, (3, 2, 8): 12, (0, 3, 4): 12, (0, 1, 6): 12, (3, 1, 9): 12, (2, 0, 9): 12, (3, 0, 9): 11, (0, 0, 6): 11, (2, 1, 9): 13, (0, 4, 4): 13, (1, 3, 6): 13, (0, 2, 6): 13, (3, 2, 9): 13, (2, 2, 8): 13, (0, 3, 5): 13, (1, 2, 7): 13, (0, 1, 7): 13, (3, 1, 10): 13, (2, 0, 10): 13, (3, 0, 10): 12, (0, 0, 7): 12, (1, 1, 8): 13, (0, 2, 7): 14, (0, 1, 8): 14, (0, 0, 8): 13, (1, 2, 8): 14, (2, 2, 9): 14, (1, 1, 9): 14, (0, 3, 6): 14, (2, 1, 10): 14, (1, 3, 7): 14, (0, 4, 5): 14, (3, 2, 10): 14, (3, 1, 11): 14, (2, 0, 11): 14, (3, 0, 11): 13, (2, 2, 10): 15, (1, 1, 10): 15, (1, 2, 9): 15, (0, 3, 7): 15, (0, 5, 5): 15, (2, 1, 11): 15, (0, 4, 6): 15, (1, 3, 8): 15, (0, 2, 8): 15, (3, 2, 11): 15, (0, 1, 9): 15, (3, 1, 12): 15, (2, 0, 12): 15, (3, 0, 12): 14, (0, 0, 9): 14, (2, 1, 12): 16, (0, 4, 7): 16, (1, 3, 9): 16, (0, 2, 9): 16, (3, 2, 12): 16, (2, 2, 11): 16, (0, 3, 8): 16, (1, 2, 10): 16, (0, 1, 10): 16, (3, 1, 13): 16, (2, 0, 13): 16, (3, 0, 13): 15, (0, 0, 10): 15, (1, 1, 11): 16, (0, 5, 6): 16, (1, 2, 11): 17, (0, 1, 11): 17, (3, 1, 14): 17, (2, 0, 14): 17, (3, 0, 14): 16, (0, 2, 10): 17, (0, 0, 11): 16, (2, 2, 12): 17, (1, 1, 12): 17, (0, 3, 9): 17, (1, 5, 6): 15, (2, 5, 7): 15, (0, 5, 7): 17, (1, 5, 7): 16, (2, 4, 8): 15, (1, 5, 8): 17, (2, 5, 8): 16, (2, 5, 9): 17, (3, 4, 9): 15, (2, 4, 9): 16, (4, 4, 10): 15, (2, 4, 10): 17, (3, 4, 10): 16, (4, 5, 11): 17, (4, 3, 11): 15, (3, 4, 11): 17, (4, 4, 11): 16, (4, 4, 12): 17, (5, 3, 12): 15, (4, 3, 12): 16, (5, 2, 13): 15, (4, 3, 13): 17, (5, 3, 13): 16, (5, 3, 14): 17, (5, 1, 14): 15, (5, 2, 14): 16, (5, 2, 15): 17, (5, 0, 15): 15, (5, 1, 15): 16, (2, 1, 13): 17, (0, 4, 8): 17, (1, 3, 10): 17}
f = {(4, 3, 12), (0, 4, 8), (0, 5, 7), (5, 1, 15), (1, 5, 7), (2, 1, 12), (0, 3, 9), (1, 3, 9), (3, 1, 14), (3, 4, 10), (5, 3, 14), (4, 4, 12), (5, 2, 15), (1, 1, 11), (5, 3, 13), (0, 1, 10), (1, 3, 10), (2, 4, 9), (0, 0, 11), (2, 0, 14), (2, 2, 11), (1, 2, 11), (0, 2, 10), (0, 4, 7), (0, 5, 6), (1, 1, 12), (2, 0, 13), (2, 5, 9), (0, 1, 11), (0, 3, 8), (4, 5, 11), (5, 0, 15), (1, 2, 10), (3, 1, 13), (3, 2, 12), (2, 4, 10), (4, 3, 13), (0, 2, 9), (0, 4, 6), (1, 5, 8), (3, 0, 14), (2, 1, 13), (2, 2, 12), (2, 5, 8), (4, 4, 11), (5, 2, 14), (3, 1, 12), (3, 4, 11)}

mapb = [[0 for i in range(6)] for j in range(6)]
for key in f:
    mapb[key[0]][key[1]] = mapb[key[0]][key[1]] + 1 

print(mapb) 













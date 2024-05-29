# simple_astar.py



 
import heapq

def astar_pq(start, goal, h_func, neighbors_func):
    """
    A* algorithm implementation using priority queue.

    :param start: Starting node.
    :param goal: Goal node.
    :param h_func: Heuristic function.
    :param neighbors_func: Function to get neighbors of a node.
    :return: Path from start to goal.
    """
    # Initialize open and closed sets
    open_set = [(0, start)]
    closed_set = set()

    # Initialize g and f values
    g = {start: 0}
    f = {start: h_func(start, goal)}

    # Initialize came_from dictionary
    came_from = {}

    while open_set:
        # Get node with lowest f value
        current = heapq.heappop(open_set)[1]

        # Check if current node is goal
        if current == goal:
            # Reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        # Move current node from open to closed set
        closed_set.add(current)

        # Get neighbors of current node
        neighbors = neighbors_func(current)

        for neighbor in neighbors:
            # Check if neighbor is already in closed set
            if neighbor in closed_set:
                continue

            # Calculate tentative g score
            tentative_g = g[current] + 1

            # Check if neighbor is not in open set or if tentative g score is lower than previous g score
            if neighbor not in g or tentative_g < g[neighbor]:
                # Update came_from, g, and f values
                came_from[neighbor] = current
                g[neighbor] = tentative_g
                f[neighbor] = g[neighbor] + h_func(neighbor, goal)

                # Add neighbor to open set if it's not already there
                heapq.heappush(open_set, (f[neighbor], neighbor))

    # If goal is not found, return None
    return None















# A* algorithm 
def astar(start, goal, h_func, neighbors_func):
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
        if current == goal:
            # Reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        # Move current node from open to closed set
        open_set.remove(current)
        closed_set.add(current)

        # Get neighbors of current node
        neighbors = neighbors_func(current)

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

    # If goal is not found, return None
    return None




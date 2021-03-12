import heapq

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

    # defining less than for purposes of heap queue
    def __lt__(self, other):
      return self.f < other.f
    
    # defining greater than for purposes of heap queue
    def __gt__(self, other):
      return self.f > other.f


def return_path(current_node):
    path = []
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    return path[::-1]  # Return reversed path


def astar_search(lab, start, end):
    # Create start and end node
    start_node = Node(None, start)
    end_node = Node(None, end)

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Heapify the open_list and add the start node
    heapq.heapify(open_list) 
    heapq.heappush(open_list, start_node)

    lab_width = (len(lab[0]) + 1) / 2
    lab_height = (len(lab) + 1) / 2

    # Adding a stop condition
    outer_iterations = 0
    max_iterations = lab_width * lab_height # // 2

    # Loop until the end is found
    while len(open_list) > 0:
        outer_iterations += 1

        if outer_iterations > max_iterations:
          print("PATH PLANNING ERROR: Giving up on pathfinding, too many iterations!")
          return return_path(current_node)       
        
        # Get the current node
        current_node = heapq.heappop(open_list)
        closed_list.append(current_node)

        # Found the end
        if current_node == end_node:
            return return_path(current_node)

        # Generate children
        children = []
        # top
        if current_node.position[1]+1 < lab_height and lab[2*current_node.position[1]+1][2*current_node.position[0]] == ' ':
            children.append(Node(current_node, (current_node.position[0], current_node.position[1]+1)))
        # bottom
        if current_node.position[1] > 0 and lab[2*current_node.position[1]-1][2*current_node.position[0]] == ' ':
            children.append(Node(current_node, (current_node.position[0], current_node.position[1]-1)))
        # right
        if current_node.position[0]+1 < lab_width and lab[2*current_node.position[1]][2*current_node.position[0]+1] == ' ':
            children.append(Node(current_node, (current_node.position[0]+1, current_node.position[1])))
        # left
        if current_node.position[0] > 0 and lab[2*current_node.position[1]][2*current_node.position[0]-1] == ' ':
            children.append(Node(current_node, (current_node.position[0]-1, current_node.position[1])))

        # Loop through children
        for child in children:
            # Skip if child is on the closed list
            if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                continue

            child.g = current_node.g + 1
            # Heuristic based on Manhattan distance
            #child.h = abs(child.position[0] - end_node.position[0]) + abs(child.position[1] - end_node.position[1])
            # Heuristic based on Euclidean distance
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Skip if child is already in the open list
            if len([open_node for open_node in open_list if child.position == open_node.position and child.g > open_node.g]) > 0:
                continue

            heapq.heappush(open_list, child)

    print("PATH PLANNING ERROR: Couldn't get a path to destination!")
    return None

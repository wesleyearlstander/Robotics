import heapq
class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def push(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def pop(self):
        return heapq.heappop(self.elements)[1]

def a_star(start, goal):
    """
    This function calculates the shortest path
    from start to goal
    and returns the nodes it took to get there
    as well as the last node it was in before
    termination for debugging purposes
    (in case there is no path found)
    It is dependent on the Node and PriorityQueue classes
    For printing the path, use print_path function
    """
    # dictionaries to keep track of the parents
    # and the cost from the start node to the current
    parents = {start: None}
    g_costs = {start: 0} # a node here is the same being closed(visited)

    open_nodes = PriorityQueue() #keeps track of unvisited nodes
    open_nodes.push(start, 0)

    current = start #just to make it global and use it at the end

    while not open_nodes.empty():
        current = open_nodes.pop()

        if current == goal: #if the goal is reached
            break

        #visit all the neighbors of the current
        for neighbor in current.links:
            g = g_costs[current] + 1 #the weight on every node is 1

            if neighbor not in g_costs or g < g_costs[neighbor]:
                g_costs[neighbor] = g
                h = current.distance(neighbor) #the heuristic is euclidean distance
                f = g + h #the final cost
                open_nodes.push(neighbor, f)
                parents[neighbor] = current

    return parents, current

def print_path(path, goal):
    """
    This function prints the path from the start node to the end node
    It takes the path and goal returned by a_star function
    It basically backtracks from the node given as goal,
    going through each parent until the node without a parent,
    the start node. Afterwards it prints from the start to the end
    """
    rot_path = []
    curr = goal
    while(curr): #change the dictionary to a list
        rot_path.append(curr)
        curr = path[curr]

     #print the list starting from the first element to the last
    for node in reversed(rot_path):
        print(node.x, node.y, sep=',')

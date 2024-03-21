import numpy as np

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.f = 0 # cost f
        self.g = 0 # cost g

    def __eq__(self, other):
        if self.position == other.position:
            return True
        
# define action
def get_action():
    # [dx, dv, cost]
    action_set = [[0,-1,1], [0,1,1], [-1,0,1], [1,0,1],
                  [1,-1,np.sqrt(2)], [1,1,np.sqrt(2)], [-1,1,np.sqrt(2)], [-1,-1,np.sqrt(2)]]
    return action_set

# check obstacle
def collision_check(omap, node):
    nx = node[0]
    ny = node[1]
    ox = node[0]
    oy = node[1]
    col = False
    for i in range(len(ox)):
        if nx == ox[i] and ny == oy[i]:
            col = True
            break
    return col

# dijkstra Algorithm
def dijkstra(start, goal, map_obstacle):

    start_node = Node(None, start)
    goal_node = Node(None, goal)

    open_list = []
    closed_list = []

    open_list.append(start_node)
    while open_list is not None:
        # Find node with lowest cost
        cur_node = open_list[0]
        cur_index = 0
        for index, node in enumerate(open_list):
            if node.f < cur_node.f:
                cur_node = node
                cur_index = index

        # If goal, return optimal path
        if cur_node.position == goal_node.position:
            opt_path = []
            node = cur_node
            while node is not None:
                opt_path.append(node.position)
                node = node.parent
            return opt_path[::-1]
        
        # If not goal, move from open list to closed list
        open_list.pop(cur_index)
        closed_list.append(cur_node)

        # Generate child candidate
        action_set = get_action()
        for action in action_set:
            child_candidate_position = (cur_node.position[0] + action[0], cur_node.position[1] + action[1])

            # If collision expected, do nothing
            if collision_check(map_obstacle, child_candidate_position):
                continue

            # If not collision, creat child node
            child = Node(cur_node, child_candidate_position)

            # If already in closed list, do nothing
            if child in closed_list:
                continue

            # If not in closed list, update open list
            child.f = cur_node.f + action[2]
            if child in open_list:
                if child.f < open_list[open_list.index(child)].f:
                    open_list[open_list.index(child)].parent = child.parent
                    open_list[open_list.index(child)].f = child.f
            else:
                open_list.append(child)
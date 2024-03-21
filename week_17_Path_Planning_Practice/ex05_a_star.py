import numpy as np
import math
import matplotlib.pyplot as plt
import random
from map_0 import map

show_animation  = True
# weight=1 : A* / weight>1 : weighted A*
weightd_a_star = 1.0

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.f = 0
        self.g = 0
        self.h = 0

    def __eq__(self, other):
        if self.position == other.position:
            return True

def get_action():
    action_set = [[1,0,1], [0,1,1], [-1,0,1], [0,-1,1], [1,1,np.sqrt(2)], [-1,1,np.sqrt(2)], [-1,-1,np.sqrt(2)], [1,-1,np.sqrt(2)]]
    return action_set

def heuristic(cur_node, goal_node):
    dx = cur_node.position[0] - goal_node.position[0]
    dy = cur_node.position[1] - goal_node.position[1]
    dist = np.sqrt(dx**2 + dy**2)
    return weightd_a_star * dist

def collision_check(omap, node):
    nx = node[0]
    ny = node[1]
    ox = omap[0]
    oy = omap[1]
    col = False
    for i in range(len(ox)):
        if nx == ox[i] and ny == oy[i]:
            col = True
            break
    return col

def a_star(start, goal, map_obstacle):

    total_cost = 0
    visited_nodes = 0

    start_node = Node(None, start)
    goal_node = Node(None, goal)

    open_list = []
    closed_list = []

    open_list.append(start_node)
    while open_list is not None:
        # Find node with lowest f cost
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
            return opt_path[::-1], total_cost, visited_nodes
        # If not goal, move from open list to closed list
        open_list.pop(cur_index)
        closed_list.append(cur_node)

        # update visited nodes count
        visited_nodes += 1

        # Generate child candidate
        action_set = get_action()
        for action in action_set:
            child_candidate_position = (cur_node.position[0] + action[0], cur_node.position[1] + action[1])
            # If collision expected, do nothing
            if collision_check(map_obstacle, child_candidate_position):
                continue
            # If not collision, create child node
            child = Node(cur_node, child_candidate_position)
            # If already in closed list, do nothing
            if child in closed_list:
                continue
            # If not in closed list, update open list
            child.g = cur_node.g + action[2]
            child.h = heuristic(child, goal_node)
            child.f = cur_node.f + child.g + child.h

            if child in open_list:
                if child.f < open_list[open_list.index(child)].f:
                    open_list[open_list.index(child)] = child
            else:
                open_list.append(child)

            # update total cost
            total_cost += action[2]

        # show graph
        if show_animation:
            plt.plot(cur_node.position[0], cur_node.position[1], 'yo', alpha=0.5)
            if len(closed_list) % 100 == 0:
                plt.pause(0.1)

def main():

    start, goal, omap = map()

    if show_animation == True:
        plt.figure(figsize=(8, 8))
        plt.plot(start[0], start[1], 'bs', markersize=7)
        plt.text(start[0], start[1] + 0.5, 'start', fontsize=12)
        plt.plot(goal[0], goal[1], 'rs', markersize=7)
        plt.text(goal[0], goal[1] + 0.5, 'goal', fontsize=12)
        plt.plot(omap[0], omap[1], '.k', markersize=10)
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("X [m]"), plt.ylabel("Y [m]")
        plt.title("A* algorithm", fontsize=20)

    opt_path, total_cost, visited_nodes = a_star(start, goal, omap)
    print("Optimal path found!")
    print("Total cost:", total_cost)
    print("Visited nodes:", visited_nodes)
    opt_path = np.array(opt_path)
    if show_animation == True:
        plt.plot(opt_path[:, 0], opt_path[:, 1], "m.-")
        plt.show()

if __name__ == "__main__":
    main()
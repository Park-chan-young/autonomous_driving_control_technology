import numpy as np
import math
import matplotlib.pyplot as plt
import random
from map_3 import map

show_animation = True
weightd_a_star = 1.0

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.f = 0
        self.g = 0
        self.h = 0

    def __eq__(self, other):
        if isinstance(other, Node):
            return self.position == other.position
        return False
        # if self.position == other.position:
        #     return True
        

# Check if position of node is same( if distance < threshold, regard as same node)
def is_same_position(node_1, node_2, epsilon_position=0.3):
    same_position = False
    diff_x = node_1.position[0] - node_2.position[0]
    diff_y = node_1.position[1] - node_2.position[1]
    distance = np.sqrt(diff_x**2 + diff_y**2)
    if distance < epsilon_position:
        same_position = True
    return same_position

def is_same_yaw(node_1, node_2, epsilon_yaw=0.2):
    sameYaw = False
    diff_yaw = node_1.position[2] - node_2.position[2]
    if diff_yaw < epsilon_yaw:
        sameYaw = True
    return sameYaw


# Action set, Moving only forward direction              
def get_action(R,Vx,delta_time_step):
    yaw_rate = Vx/R
    distance_travel = Vx*delta_time_step
    # action_set = [turn right, left, half right, half left, go straight]
    # action_set[] = [yaw_rate, delta_time_step, cost]
    action_set = [[yaw_rate, delta_time_step, distance_travel], 
                  [-yaw_rate, delta_time_step, distance_travel],
                  [yaw_rate/2, delta_time_step, distance_travel],
                  [-yaw_rate/2, delta_time_step, distance_travel],
                  [0.0, delta_time_step, distance_travel]]
    return action_set


# Vehicle movement
def vehicle_move(position_parent, yaw_rate, delta_time, Vx):
    x_parent = position_parent[0]
    y_parent  = position_parent[1]
    yaw_parent = position_parent[2]
    # if yaw_rate != 0 (left or right turn)
    if abs(yaw_rate) > 1e-5:
        R = Vx / yaw_rate
        delta_yaw = yaw_rate * delta_time
        yaw_child = yaw_parent + delta_yaw

        delta_x = R*np.sin(yaw_child) - R*np.sin(yaw_parent)
        delta_y = R*np.cos(yaw_parent) - R*np.cos(yaw_child)
        x_child = x_parent + delta_x
        y_child = y_parent + delta_y
    # move straight
    d = Vx * delta_time
    delta_x = d * np.cos(yaw_parent)
    delta_y = d * np.sin(yaw_parent)
    x_child = x_parent + delta_x
    y_child = y_parent + delta_y
    yaw_child = yaw_parent
    # yaw processing
    if yaw_child > 2*np.pi:
        yaw_child = yaw_child - 2*np.pi
    if yaw_child < 0:
        yaw_child = yaw_child + 2*np.pi
    # return position : [x, y, yaw]
    return [x_child, y_child, yaw_child]


def collision_check(position_parent, yaw_rate, delta_time_step, obstacle_list, Vx):
    col = False
    # Vehicle movement
    pose_child = vehicle_move(position_parent, yaw_rate, delta_time_step, Vx)
    x_child, y_child, yaw_child = pose_child

    # Check if the new position overlaps with any of the obstacles
    for obstacle in obstacle_list:
        obstacle_x, obstacle_y, obstacle_radius = obstacle
        distance_to_obstacle = np.sqrt((x_child - obstacle_x)**2 + (y_child - obstacle_y)**2)
        # If too close, collision occurred
        if distance_to_obstacle < obstacle_radius + 0.3:
            col = True
            break

    return col  # No collision


# Check if the node is in the searching space
def is_not_in_searching_space(position_child, space):
    x, y = position_child[0], position_child[1]
    x_min, x_max, y_min, y_max = space

    if x < x_min or x > x_max or y < y_min or y > y_max:
        return True  # Node is outside the searching space
    else:
        return False  # Node is inside the searching space


def heuristic(cur_node, goal_node):
    dx = cur_node.position[0] - goal_node.position[0]
    dy = cur_node.position[1] - goal_node.position[1]
    dist = np.sqrt(dx**2 + dy**2)
    return weightd_a_star * dist


def hybrid_a_star(start, goal, space, obstacle_list, R, Vx, delta_time_step):
    
    start_node = Node(None, start)
    goal_node = Node(None, goal)
    
    open_list = []
    closed_list = []
    
    open_list.append(start_node)
    while open_list:
        # Find node with lowest cost
        cur_node = open_list[0]
        cur_index = 0
        for index, node in enumerate(open_list):
            if node.f < cur_node.f:
                cur_node = node
                cur_index = index
        # If goal, return optimal path
        if (is_same_position(cur_node, goal_node) and is_same_yaw(cur_node, goal_node)):
            opt_path=[]
            node = cur_node
            while node is not None:
                opt_path.append(node.position[0,1])
                node = node.parent
            return opt_path[::-1] 
        # If not goal, move from open list to closed list
        open_list.pop(cur_index)
        closed_list.append(cur_node)
        
        # Generate child candidate
        action_set = get_action(R, Vx, delta_time_step)
        for action in action_set:
            child_candidate_position = vehicle_move(cur_node.position, action[0], action[1], Vx)
            # If not in searching space, do nothing
            if is_not_in_searching_space(child_candidate_position, space):
                continue
            # If collision expected, do nothing
            if collision_check(cur_node.position, action[0], action[1], obstacle_list, Vx):
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
        
        # show graph
        if show_animation:
            plt.plot(cur_node.position[0], cur_node.position[1], 'yo', alpha=0.5)
            if len(closed_list) % 100 == 0:
                plt.pause(0.1)


def main():

    start, goal, obstacle_list, space = map()
    if show_animation == True:
        theta_plot = np.linspace(0,1,101) * np.pi * 2
        plt.figure(figsize=(8,8))
        plt.plot(start[0], start[1], 'bs',  markersize=7)
        plt.text(start[0], start[1]+0.5, 'start', fontsize=12)
        plt.plot(goal[0], goal[1], 'rs',  markersize=7)
        plt.text(goal[0], goal[1]+0.5, 'goal', fontsize=12)
        # len(obstacle_list) = 18
        for i in range(len(obstacle_list)):
            x_obstacle = obstacle_list[i][0] + obstacle_list[i][2] * np.cos(theta_plot)
            y_obstacle = obstacle_list[i][1] + obstacle_list[i][2] * np.sin(theta_plot)
            plt.plot(x_obstacle, y_obstacle,'k-')
        plt.axis(space)
        plt.grid(True)
        plt.xlabel("X [m]"), plt.ylabel("Y [m]")
        plt.title("Hybrid a star algorithm", fontsize=20)

    opt_path = hybrid_a_star(start, goal, space, obstacle_list, R=5.0, Vx=2.0, delta_time_step=0.5)
    opt_path = np.array(opt_path)
    if show_animation == True:
        plt.plot(opt_path[:, 0], opt_path[:, 1], "m.-")
        plt.show()


if __name__ == "__main__":
    main()

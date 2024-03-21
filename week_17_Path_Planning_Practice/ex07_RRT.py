import numpy as np
import matplotlib.pyplot as plt
from map_4 import map

class Node(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

    def set_parent(self, parent):
        self.parent = parent


class RRT(object):
    def __init__(self, start, goal, space, obstacle_list, success_dist_thres=1.0):
        self.start_node = Node(start[0], start[1])  # node (x, y)
        self.goal_node = Node(goal[0], goal[1])  # node (x, y)
        self.space = space  # (min_x, max_x, min_y, max_y)
        self.obstalce_list = obstacle_list  # list of (x, y ,r)
        self.node_list = []

        # options
        self.max_iter = 5000
        self.goal_sample_rate = 0.1
        self.min_u = 1.0
        self.max_u = 3.0
        self.success_dist_thres = success_dist_thres
        self.collision_check_step = 0.2
        self.stepsize = 0.5

    def visualize_tree(self, node_size=5):
        for node in self.node_list:
            if node.parent is not None:
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], 'go-', markersize=node_size)

        plt.text(self.start_node.x, self.start_node.y, 'start', fontsize=12, ha='right', va='bottom', color='blue')
        plt.text(self.goal_node.x, self.goal_node.y, 'goal', fontsize=12, ha='left', va='top', color='green')

    def plan(self):
        self.node_list = [self.start_node]
        for i in range(self.max_iter):
            # Get random node
            rand_node = self.get_random_node()
            # Find neareast node
            nearest_node = self.find_nearest_node(self.node_list, rand_node)
            # Create new node
            u = self.stepsize*self.get_random_input(self.min_u, self.max_u)
            new_node = self.create_child_node(nearest_node, rand_node, u)
            # Collision check(Node, Path)
            node_collide = self.is_collide(new_node, self.obstalce_list)
            if node_collide:
                continue
            collide = self.is_path_collide(nearest_node, new_node, self.obstalce_list, self.collision_check_step)
            if collide:
                continue
            # Add to tree
            new_node.set_parent(nearest_node)
            self.node_list.append(new_node)
            self.visualize_tree()
            # Goal check
            goal_reached = self.check_goal(new_node, self.success_dist_thres)
            if goal_reached:
                print(" [-] GOAL REACHED")
                return self.backtrace_path(new_node)
        return None

    @staticmethod
    def is_same_node(node1, node2):
        same_node = False
        if (node1.x == node2.x and node1.y == node2.y):
            same_node = True
        return same_node

    def backtrace_path(self, node):
        current_node = node
        path = [current_node]
        reached_start_node = self.is_same_node(current_node, self.start_node)
        while not reached_start_node:
            current_node = current_node.parent
            path.append(current_node)
            reached_start_node = self.is_same_node(
                current_node, self.start_node)
        return path[::-1]

    def get_random_node(self):
        if np.random.rand() < self.goal_sample_rate:
            return self.goal_node
        else:
            x_rand = np.random.uniform(self.space[0], self.space[1])
            y_rand = np.random.uniform(self.space[2], self.space[3])
            rand_node = Node(x_rand, y_rand)
            return rand_node

    def check_goal(self, node, success_dist_thres):
        dx = node.x - self.goal_node.x
        dy = node.y - self.goal_node.y
        dist = np.sqrt(dx**2 + dy**2)
        check = False
        if (dist < success_dist_thres):
            check = True
        return check

    @staticmethod
    def create_child_node(nearest_node, rand_node, u):
        dx = rand_node.x - nearest_node.x
        dy = rand_node.y - nearest_node.y
        d = np.sqrt(dx**2 + dy**2)
        if d > u:
            theta = np.arctan2(dy, dx)
            new_x = nearest_node.x + u * np.cos(theta)
            new_y = nearest_node.y + u * np.sin(theta)
        else:
            new_x = rand_node.x
            new_y = rand_node.y
        new_node = Node(new_x, new_y)
        return new_node

    @staticmethod
    def get_random_input(min_u, max_u):
        # Code
        return np.random.uniform(min_u, max_u)

    @staticmethod
    def find_nearest_node(node_list, rand_node):
        min_dist = float('inf')
        min_index = None
        for i, node in enumerate(node_list):
            dx = node.x - rand_node.x
            dy = node.y - rand_node.y
            dist = np.sqrt(dx**2 + dy**2)
            if dist < min_dist:
                min_dist = dist
                min_index = i
        return node_list[min_index]

    @staticmethod
    def is_collide(node, obstacle_list):
        col = False
        for obstacle in obstacle_list:
            dx = node.x - obstacle[0]
            dy = node.y - obstacle[1]
            dist = np.sqrt(dx**2 + dy**2)
            if dist <= obstacle[2]:
                col = True
        return col

    @staticmethod
    def is_path_collide(node_from, node_to, obstacle_list, check_step=0.2):
        col = False
        dx = node_to.x - node_from.x
        dy = node_to.y - node_from.y
        dist = np.sqrt(dx**2 + dy**2)
        if dist <= 0.001:
            col = False

        num_steps = int(dist / check_step)
        for i in range(num_steps):
            x_check = node_from.x + dx * (i / num_steps)
            y_check = node_from.y + dy * (i / num_steps)
            node_check = Node(x_check, y_check)
            if RRT.is_collide(node_check, obstacle_list):
                col = True
        return col


if __name__ == "__main__":
    start, goal, space, obstacle_list = map()

    success_dist_thres = 1.0
    rrt = RRT(start, goal, space, obstacle_list, success_dist_thres)
    path = rrt.plan()
    if path is not None:
        for node in path:
            print(" [-] x = %.2f, y = %.2f " % (node.x, node.y))

        # draw result
        _t = np.linspace(0, 2*np.pi, 30)
        for obs in obstacle_list:
            x, y, r = obs
            _x = x + r * np.cos(_t)
            _y = y + r * np.sin(_t)

            plt.plot(_x, _y, 'k-')

        goal_x = goal[0] + success_dist_thres * np.cos(_t)
        goal_y = goal[1] + success_dist_thres * np.sin(_t)
        plt.plot(goal_x, goal_y, 'g--')

        for i in range(len(path)-1):
            node_i = path[i]
            node_ip1 = path[i+1]
            plt.plot([node_i.x, node_ip1.x], [node_i.y, node_ip1.y], 'r.-')

        plt.axis("equal")
        plt.grid(True)
        plt.show()
    else:
        print(" [-] FAIL TO REACH GOAL")

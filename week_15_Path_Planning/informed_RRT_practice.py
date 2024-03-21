import numpy as np
import matplotlib.pyplot as plt
import networkx as nx


class RRTStar(object):
    def __init__(self, start, goal, config):
        self.G = nx.DiGraph() # Directional Graph
        # Add Start node [(Node ID, {Cost, Position})]
        self.G.add_nodes_from([(-1, {'cost' : 0, 'x' : start[0], 'y' : start[1]})])
        self.start = start
        self.goal = goal
        self.config = config

        self.G.nodes[node_id]['cost']
        self.G.nodes[node_id]['x']
        self.G.nodes[node_id]['y']
        self.G.add_node(node_id, x=x, y=y)
        self.G.add_edge(node_from_id, node_to_id)
        self.G.remove_edge(node_from_id, node_to_id)
        parents = list(self.G.predecessors(node_id))

    def plan(self):

        is_first_node = True
        goal_node_id = None
        
        for i in range(1000):
        ## Create Random Node
            rand_node = rrt_star.sample_free(obstacles, space)
            # plt.plot(rand_node[0], rand_node[1], '.')
        ## Find Nearest Node
            nearest_node_id = rrt_star.get_nearest(rand_node)
            nearest_node = rrt_star.get_node(nearest_node_id)
        ## Connect new node to the nearest node
            new_node = rrt_star.steer(nearest_node, rand_node)
            # plt.plot(new_node[0], new_node[1], 's')
        ## Check Collision of the new node
            if rrt_srat.is_collision_free(nearest_node, new_node, obstacles):
        ## Find adjacent nodes
                nearest_node_ids = rrt_star.get_near_node_ids(new_node, draw=True)
                rrt_star.add_node(i, new_node[0], new_node[1])
                if is_first_node:
                    rrt_star.add_edge(-1, i)
                    is_first_node = False
                plt.plot(new_node[0], new_node[1], 's')

                min_node_id = nearest_node_id
                min_cost = rrt_star.get_node_cost(nearest_node_id) + rrt_star.get_distance(i, nearest_node_id)
        ## Find a node with minimum cost among adjacent nodes
                for near_node_id in near_node_ids:
                    near_node = rrt_star.get_node(near_node_id)
                    if rrt_star.is_collision_free(near_node, new_node, obstacles):
                        cost = rrt_star.get_node_cost(near_node_id) + rrt_star.get_distance(near_node_id, i)
                        if cost < min_cost:
                            min_node_id = near_node_id
                            min_cost = cost

                rrt_star.set_node_cost(i, min_cost)
                rrt_star.add_edge(min_node_id, i)
        ## Rewire the tree with adjacent nodes
                for near_node_id in near_node_ids:
                    near_node = rrt_star.get_node(near_node_id)
                    if rrt_star.is_collision_free(new_node, near_node, obstacles):
                        cost = rrt_star.get_node_cost(i) + rrt_star.get_distance(i, near_node_id)
                        if cost < rrt_star.get_node_cost(near_node_id):
                            parent_node_id = rrt_star.get_parent(near_node_id)
                            if parent_node_id is not None:
                                rrt_star.remove_edge(parent_node_id, near_node_id)
                                rrt_star.add_dege(i, near_node_id)
        ## Check Goal
                if rrt_star.check_goal_by_id(i):
                    goal_node_id = i
                    break
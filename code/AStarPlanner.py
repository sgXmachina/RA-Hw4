import Queue
import random
from SimpleEnvironment import Control,Action

'''
Source from: https://en.wikipedia.org/wiki/A*_search_algorithm
'''

class AStarPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict() # holds the distance from start

    def Plan(self, start_config, goal_config):

        plan = []

        # TODO: Here you will implement the AStar planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        if(self.visualize): # initialize plot
            self.planning_env.InitializePlot(goal_config)

        # implement a priority queue
        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        class Node(object):
            def __init__(self, node_id, config, g_score, goal_id, planning_env, distance_from_start = float("inf")):
                self.node_id = node_id
                self.config = config
                self.g_score = g_score
                self.h_score = planning_env.ComputeHeuristicCost(node_id, goal_id)
                self.f_score = self.g_score + self.h_score
                self.distance_from_start = distance_from_start
            def __cmp__(self, other):
                return cmp(self.f_score, other.f_score)

        priority_queue = Queue.PriorityQueue()

        # Open queue
        priority_queue.put(Node(start_id, start_config, 0, goal_id, self.planning_env, 0)) # input start node to queue

        # Initialize lists
        closed_set = []
        open_set = []
        g_scores = dict()
        f_scores = dict()
        current_id = start_id
        open_set.append(start_id)
        g_scores[start_id] = 0
        f_scores[start_id] = self.planning_env.ComputeHeuristicCost(start_id, goal_id)

        # initialize flags and counters
        found_goal = False

        while len(open_set) != 0:
            # Get the element with the lowest f_score
            minn = float("inf")
            min_node = None
            min_idx = 0
            for i in xrange(0, len(open_set)):
                try:
                    f_score = f_scores[open_set[i]]
                except (KeyError):
                    pass
                if f_score < minn:
                    minn = f_score
                    min_node = open_set[i]
                    min_idx = i
            curr_id = min_node

            # Remove element from open set
            open_set.pop(min_idx)

            # Check to see if you are at goal
            if(curr_id == goal_id):
                found_goal = True
                break

            # Add node to closed set
            if(curr_id not in closed_set):
                closed_set.append(curr_id)

            # Find a non-visited successor to the current_id
            successors = self.planning_env.GetSuccessors(curr_id)
            for successor_action in successors:

                for i in xrange(len(successor_action.footprint)):

                    successor = self.planning_env.discrete_env.ConfigurationToNodeId(successor_action.footprint[i])
                    if(successor in closed_set):
                        continue
                    else:
                        # Calculate the tentative g score
                        successor_config = self.planning_env.discrete_env.NodeIdToConfiguration(successor)
                        g_score = g_scores[curr_id] + self.planning_env.ComputeDistance(curr_id, successor)
                        if successor not in open_set:
                            # Add to open set
                            open_set.append(successor)
                        elif g_score >= g_scores[successor]:
                            continue

                        # Update g and f scores
                        g_scores[successor] = g_score
                        f_scores[successor] = g_score + self.planning_env.ComputeHeuristicCost(successor, goal_id)

                        # Store the parent and child
                        self.nodes[successor] = [successor_action,curr_id]

                        if self.visualize: # Plot the edge
                            pred_config = self.planning_env.discrete_env.NodeIdToConfiguration(curr_id)
                            succ_config = self.planning_env.discrete_env.NodeIdToConfiguration(successor)
                            self.planning_env.PlotEdge(pred_config, succ_config)

        if found_goal:
            # Find the path in reverse from goal
            curr_id = goal_id
            while(curr_id != start_id):
                curr_confg = self.planning_env.discrete_env.NodeIdToConfiguration(curr_id)
                plan.append(self.nodes[curr_id][0])
                curr_id = self.nodes[curr_id][1] # Get the vertex opposite the edge of the current id

            # Whenever the current id is the start id, append start id
            #plan.append(self.nodes[start_id][0])

            return plan[::-1] # reverse the plan

        else:
            print("Failed to Find Goal")
            return [] # Failure

""" The Path Selector takes the graph from the collision checker and searches over for the best path.
    The first action of the path is passed to the Trajectory Generator. """

"""Assumptions:
    dynamic programming approach:
        all paths are clear of obstacles
        paths are handed over all at once in a data stucture like a list of lists of actions
        this path selector iterates over all the lists of actions and assigns them costs based on cost structure
        returning the smallest costing path to the trajectory planner

    aStar approach:
        lattice is given
        lattice has been trimmed of collision causing actions
        path selector assigns costs and heuristics based on cost structure with an aStar approach to the lattice graph
        returns the list of actions to be the least costly    

    the lattice is made of states and actions
    the lattice is made of a data structure of wrappers (nodes) for state action combinations holding the previous state/s, the action/s, and the current state
    I will need to add a wrapper around the nodes to store the h and g values, and create comparitiors to compare these nodes
    the trajectory planner only wants the list of actions to get to each state

    Costs:
        social cost: (whatever this means, maybe lateral distance to other car, isn't the idea to get the other car to yield and for our car to be more decisive?)
            lateral distance to the other car, highest in the other car's blindspot, drops off until our car is in front or behind the car with preportional ramps
            dependent on the other car's acceleration
            higher when lange change is needed
        speed limit: ramping costs for being 15 mph above or below speed limit (vel)


    Heuristics:
        ???

        """

class PathSelector:
    def __init__(self):
        self.lattice_graph = None

    def search_path(self):
        pass

import numpy as np

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution
        self.resolution = resolution  #+ 0.0 # ensure it is a double

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        for idx in range(self.dimension):
            self.num_cells[idx] = np.ceil((upper_limits[idx] - lower_limits[idx])/resolution[idx])


    def ConfigurationToNodeId(self, config):
        
        # TODO:
        # This function maps a node configuration in full configuration
        # space to a node in discrete space
        #
        return self.GridCoordToNodeId(self.ConfigurationToGridCoord(config))

    def NodeIdToConfiguration(self, nid):
        
        # TODO:
        # This function maps a node in discrete space to a configuraiton
        # in the full configuration space
        #
        return self.GridCoordToConfiguration(self.NodeIdToGridCoord(nid))
        
    def ConfigurationToGridCoord(self, config):
        
        # TODO:
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #
        coord = [0] * self.dimension
        for i in xrange(0, len(coord)):
            start = self.lower_limits[i] # start of the configuration space
            coord[i] = np.around((config[i] - start)/ self.resolution[i])
        return coord

    def GridCoordToConfiguration(self, coord):
        
        # TODO:
        # This function smaps a grid coordinate in discrete space
        # to a configuration in the full configuration space
        #
        config = [0] * self.dimension
        for i in xrange(0, len(coord)):
            start = self.lower_limits[i] # start of the configuration space
            grid_step = self.resolution[i] * coord[i] # step from the coordinate in the grid
            half_step = self.resolution[i] / 2 # To get to middle of the grid
            config[i] = start + grid_step # + half_step
        return config

    def GridCoordToNodeId(self,coord):
        
        # TODO:
        # This function maps a grid coordinate to the associated
        # node id 
        node_id = 0
        for i in xrange(len(coord) - 1, -1, -1):
            # Get product of the grid space maximums
            prod = 1
            for j in xrange(0, i):
                prod  = prod * self.num_cells[j]
            # Add the product to the sum
            node_id = node_id + coord[i] * prod
        return (node_id)


    def NodeIdToGridCoord(self, node_id):

        # TODO:
        # This function maps a node id to the associated
        # grid coordinate
        coord = [0] * self.dimension
        for i in xrange(len(coord) - 1, -1, -1):
            # Get the product of the grid space maximums
            prod = 1
            for j in xrange(0, i):
                prod = prod * self.num_cells[j]
            coord[i] = np.floor(node_id / prod)
            node_id = node_id - (coord[i] * prod)
        return coord
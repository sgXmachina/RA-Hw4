import numpy, openravepy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment
import copy

class Control(object):
    def __init__(self, omega_left, omega_right, duration):
        self.ul = omega_left
        self.ur = omega_right
        self.dt = duration

class Action(object):
    def __init__(self, control, footprint):
        self.control = control
        self.footprint = footprint

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.herb = herb
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5., -numpy.pi], [5., 5., numpy.pi]]
        lower_limits, upper_limits = self.boundary_limits
        self.discrete_env = DiscreteEnvironment(resolution, lower_limits, upper_limits)

        self.resolution = resolution
        self.ConstructActions()

    def GenerateFootprintFromControl(self, start_config, control, stepsize=0.01):

        # Extract the elements of the control
        ul = control.ul
        ur = control.ur
        dt = control.dt
        # Initialize the footprint
        config = start_config.copy()
        footprint = [numpy.array([0., 0., config[2]])]
        timecount = 0.0
        while timecount < dt:
            # Generate the velocities based on the forward model
            xdot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.cos(config[2])
            ydot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.sin(config[2])
            tdot = self.herb.wheel_radius * (ul - ur) / self.herb.wheel_distance
                
            # Feed forward the velocities
            if timecount + stepsize > dt:
                stepsize = dt - timecount
            config = config + stepsize*numpy.array([xdot, ydot, tdot])
            if config[2] > numpy.pi:
                config[2] -= 2.*numpy.pi
            if config[2] < -numpy.pi:
                config[2] += 2.*numpy.pi

            footprint_config = config.copy()
            footprint_config[:2] -= start_config[:2]
            footprint.append(footprint_config)

            timecount += stepsize
            
        # Add one more config that snaps the last point in the footprint to the center of the cell
        nid = self.discrete_env.ConfigurationToNodeId(config)
        snapped_config = self.discrete_env.NodeIdToConfiguration(nid)
        snapped_config[:2] -= start_config[:2]
        footprint.append(snapped_config)

        return footprint

    def PlotActionFootprints(self, idx):

        actions = self.actions[idx]
        fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        
        for action in actions:
            xpoints = [config[0] for config in action.footprint]
            ypoints = [config[1] for config in action.footprint]
            pl.plot(xpoints, ypoints, 'k')
                     
        pl.ion()
        pl.show()

        

    def ConstructActions(self):

        # Actions is a dictionary that maps orientation of the robot to
        #  an action set
        self.actions = dict()
              
        wc = [0., 0., 0.]
        grid_coordinate = self.discrete_env.ConfigurationToGridCoord(wc)
        
        dur=0.2
        turnDur= (3.141 / 1.6)
        controls = [[ 1,1,dur],
                [-1,-1,dur],
                [-1,1,turnDur],
                [1,-1,turnDur],
                [-1,1,2*turnDur],
                [1,-1,2*turnDur],
                [1,3,dur],
                [3,1,dur],
                [1,15,dur],
                [15,1,dur],
                [1,30,dur],
                [30,1,dur],
                [ 1,1,5*dur],
                [-1,-1,5*dur],
                [1,3,5*dur],
                [3,1,5*dur]]
                #[1,10,5*dur],
                #[10,1,5*dur]]
        # Iterate through each possible starting orientation
        for idx in range(int(self.discrete_env.num_cells[2])):
            self.actions[idx] = []
            grid_coordinate[2] = idx
            start_config = numpy.asarray(self.discrete_env.GridCoordToConfiguration(grid_coordinate))
            for i in range(len(controls)):
                ctrl=Control(controls[i][0],controls[i][1],controls[i][2])
                footprint= self.GenerateFootprintFromControl(start_config,ctrl)
                self.actions[idx].append(Action(ctrl,footprint))
        #for j in range(len(self.actions)):
        #    self.PlotActionFootprints(j)

    def GetSuccessors(self, node_id):

        successors = []
        lower_limits, upper_limits = self.boundary_limits
        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids and controls that represent the neighboring
        #  nodes
        start_config = numpy.asarray(self.discrete_env.NodeIdToConfiguration(node_id))
        orientation= self.discrete_env.NodeIdToGridCoord(node_id)[2]
        # if orientation==8:
        #     print("")
        for i in (self.actions[orientation]):
            footprint = self.GenerateFootprintFromControl(start_config,i.control)
            # iterate through footprint and check for range & collision violations
            break_flag=0
            for delta_config in footprint:
            	config=numpy.asarray([0.,0.,0.])
            	config[0]=start_config[0]+delta_config[0]
            	config[1]=start_config[1]+delta_config[1]
            	config[2]=delta_config[2]
            	if config[0]<=lower_limits[0] or config[0]>=upper_limits[0] or \
            	config[1]<=lower_limits[1] or config[1]>=upper_limits[1] or \
            	config[2]<=lower_limits[2] or config[2]>=upper_limits[2] or \
            	self.CollisionChecker(config):

#            		print "rejectedConfig{"
#            		print config
#            		print "}"
            		break_flag=1
            		break
            if break_flag == 1:	
            	continue
#            print config
            footprint_abs = copy.deepcopy(footprint)
            for f in range(len(footprint_abs)):
            	footprint_abs[f][0] += start_config[0] 
            	footprint_abs[f][1] += start_config[1] 
            successors.append(Action(i.control,footprint_abs))
#        print "done"
        return successors

    def ComputeDistance(self, start_id, end_id):
        dist = 0
        start_config = numpy.array(self.discrete_env.NodeIdToConfiguration(start_id))
        end_config = numpy.array(self.discrete_env.NodeIdToConfiguration(end_id))


        dist = ((start_config[0] - end_config[0])**2 + (start_config[1] - end_config[1])**2 + (0.2*(start_config[2] - end_config[2]))**2 )**0.5
        # dist= numpy.linalg.norm(start_config - end_config, 2)
        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        '''
        D=1
        D2= 1.414
        #computing the octile distance
        start_config = numpy.array(self.discrete_env.NodeIdToConfiguration(start_id))
        end_config = numpy.array(self.discrete_env.NodeIdToConfiguration(goal_id))

        dx = abs(start_config[0] - end_config[0])
        dy = abs(start_config[1] - end_config[1])
        cost = D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)
        '''
        cost = self.ComputeDistance(start_id, goal_id)
        return cost

    def CollisionChecker(self, node_config):
        #config = self.discrete_env.GridCoordToConfiguration(node_coord)
        config = node_config
        config_pose = numpy.array([[ 1, 0,  0, config[0]], 
                                   [ 0, 1,  0, config[1]], 
                                   [ 0, 0,  1, 0], 
                                   [ 0, 0,  0, 1]])
        with self.robot.GetEnv():
            self.robot.SetTransform(config_pose)
        check = self.robot.GetEnv().CheckCollision(self.robot)
        return check
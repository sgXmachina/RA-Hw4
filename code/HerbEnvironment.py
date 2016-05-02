import numpy 
import pdb
import time

class HerbEnvironment(object):
    
    def __init__(self, herb):
        self.robot = herb.robot
        self.herb = herb

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.6], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        self.table = table

        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
        
        #self.GenerateRandomConfiguration()
        # goal sampling probability
        # self.ComputeDistance
        self.p = 0.2

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        

    def GenerateRandomConfiguration(self):
        config = [0] * len(self.robot.GetActiveDOFIndices())
        print self.robot.GetActiveDOFIndices()
        
        legalConfig = False
        while (not legalConfig):
            # Generate random config
            #config=numpy.random.rand(len(self.robot.GetActiveDOFIndices()))
            #config=config - .5
            #config = 4 * config 
            # Set joints to random config
            lower, upper = self.robot.GetActiveDOFLimits()
            for i in range(7):
                config[i] = numpy.random.uniform(lower[i], upper[i])
            #pdb.set_trace()
            self.robot.SetDOFValues(config,self.robot.GetActiveDOFIndices(),checklimits=1)
            # Check for collisions
            # If legal exit loop
            selfC = self.robot.CheckSelfCollision()
            env = self.robot.GetEnv()
            envC = env.CheckCollision(self.robot,self.table)

            # print "Self C: " + str(selfC) + " env C: " + str(envC) 
            if ((not selfC) and (not envC)):
                legalConfig=True

        return numpy.array(config)


    
    def ComputeDistance(self, start_config, end_config):
        
        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        #
        dist = numpy.linalg.norm(start_config-end_config)       

        return dist


    def ExtendN(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #

        increment_length = 0.01
        step_size = 0.3
        
        current_position = numpy.array([0,0,0,0,0,0,0])

        dist = self.ComputeDistance(start_config,end_config)
        unit_vector = (end_config - start_config)/dist
        increment_dist = unit_vector*increment_length
        interpolate_num = int(dist/increment_length)

        for i in range(interpolate_num):
            
            config = start_config + increment_dist*(i+1)
            #pdb.set_trace()
            self.robot.SetDOFValues(config,self.robot.GetActiveDOFIndices(),checklimits=1)

            selfC = self.robot.CheckSelfCollision()
            env = self.robot.GetEnv()
            envC = env.CheckCollision(self.robot,self.table)

            if ((selfC) or (envC)):
                #One can either move until obstacle (CONNECT), or just discard it (EXTEND)
                #return current_position - increment_dist
                self.robot.SetDOFValues(start_config,self.robot.GetActiveDOFIndices(),checklimits=1)
                return None
        #if numpy.linalg.norm(current_position - start_config)> step_size:
        #    return start_config + step_size*unit_vector
        #else:
        return end_config

    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #

        increment_length = 0.01
        step_size = 0.3
        
        current_position = numpy.array([0,0,0,0,0,0,0])

        dist = self.ComputeDistance(start_config,end_config)
        unit_vector = (end_config - start_config)/dist
        increment_dist = unit_vector*increment_length
        interpolate_num = int(dist/increment_length)

        for i in range(interpolate_num):
            
            config = start_config + increment_dist*(i+1)
            #pdb.set_trace()
            self.robot.SetDOFValues(config,self.robot.GetActiveDOFIndices(),checklimits=1)

            selfC = self.robot.CheckSelfCollision()
            env = self.robot.GetEnv()
            envC = env.CheckCollision(self.robot,self.table)

            if ((selfC) or (envC)):
                #One can either move until obstacle (CONNECT), or just discard it (EXTEND)
                #return current_position - increment_dist
                self.robot.SetDOFValues(start_config,self.robot.GetActiveDOFIndices(),checklimits=1)
                return None
        #if numpy.linalg.norm(current_position - start_config)> step_size:
        #    return start_config + step_size*unit_vector
        #else:
        return end_config

    def ShortenPath(self, path, timeout=5.0):
        
        start = time.time()
        while (time.time()-start)<50:
            increment_length = 0.1
            goal_config = path[-1]
            l = len(path)
            new_path = []
            new_path.append(path[0])
            for i in range(l):
                end_config = path[i+1]
                start_config = path[i] 
                dist = self.ComputeDistance(start_config,end_config)
                unit_vector = (end_config - start_config)/dist
                increment_dist = unit_vector*increment_length
                interpolate_num = int(dist/increment_length)
                new_path.append(path[i])
                for j in range(interpolate_num):
                    current_position = start_config + increment_dist*(j+1)
                    check = self.ExtendN(current_position,goal_config)

                    if check != None:
                        #pdb.set_trace()
                        new_path.append(current_position)
                        new_path.append(goal_config)
                        break
                else:
                    continue
                break
            return new_path
        return path 
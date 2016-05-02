import logging, numpy, openravepy
import pdb
import time
class GraspPlanner(object):

    def __init__(self, robot, base_planner, arm_planner):
        self.robot = robot
        self.base_planner = base_planner
        self.arm_planner = arm_planner
        self.gmodel = 0

            
    def GetBasePoseForObjectGrasp(self, obj):

        # Load grasp database
        gmodel = openravepy.databases.grasping.GraspingModel(self.robot, obj)
        if not gmodel.load():
            gmodel.autogenerate()

        base_pose = None
        grasp_config = None
       
        ###################################################################
        # TODO: Here you will fill in the function to compute
        #  a base pose and associated grasp config for the 
        #  grasping the bottle
        ###################################################################

        theta=0
        base_pose = numpy.array([[numpy.cos(theta), -numpy.sin(theta), 0, -0.1],
                      [numpy.sin(theta),  numpy.cos(theta), 0,  0.0],
                      [0.              ,  0.              , 1,  0.  ],
                      [0.              ,  0.              , 0,  1.  ]])

        init_pose = self.base_planner.planning_env.herb.GetCurrentConfiguration()
        theta=init_pose[2]
        init_pose = numpy.array([[numpy.cos(theta), -numpy.sin(theta), 0, init_pose[0]],
                      [numpy.sin(theta),  numpy.cos(theta), 0,  init_pose[1]],
                      [0.              ,  0.              , 1,  0.  ],
                      [0.              ,  0.              , 0,  1.  ]])

        r = self.base_planner.planning_env.herb.robot

        irmodel=openravepy.databases.inversereachability.InverseReachabilityModel(robot = r)
        #irmodel.autogenerate()
        irmodel.load()        

        r.SetTransform(base_pose)

        self.gmodel = gmodel
        validgrasps, validindices = gmodel.computeValidGrasps(returnnum = 7)
        
        # select best grasp from valid grasp list
        validgrasps = self.order_grasps(validgrasps)
        validgrasp = validgrasps[0]
        gmodel.showgrasp(validgrasp)

        Tconfig = gmodel.getGlobalGraspTransform(validgrasp, collisionfree = True)

        densityfn,samplerfn,bounds = irmodel.computeBaseDistribution(Tconfig)

        # Inverse reachability to determine base position
        goals = []
        numfailures = 0
        N = 5
        m = self.arm_planner.planning_env.herb.manip
        fo = openravepy.IkFilterOptions.CheckEnvCollisions
        with r:
            while len(goals) < N:
                poses,jointstate = samplerfn(N-len(goals))
                for pose in poses:
                    r.SetTransform(pose)
                    r.SetDOFValues(*jointstate)
                    # validate that base is not in collision
                    if not m.CheckIndependentCollision(openravepy.CollisionReport()):
                        q = m.FindIKSolution(Tconfig,filteroptions=fo)
                        if q is not None:
                            values = r.GetDOFValues()
                            values[m.GetArmIndices()] = q
                            goals.append((Tconfig,pose,values))
                        elif m.FindIKSolution(Tconfig,0) is None:
                            numfailures += 1
        
        #r.SetTransform(goals[0][1])
        r.SetTransform(base_pose)

        grasp_config = self.arm_planner.planning_env.herb.manip.FindIKSolution(Tconfig, filteroptions=fo)

        r.SetTransform(init_pose)
        theta=0
        #base_pose = goals[0][1]
        base_pose = numpy.array([-0.1,0.0,0]);

        return base_pose, grasp_config

    def PlanToGrasp(self, obj):

        # Next select a pose for the base and an associated ik for the arm
        base_pose, grasp_config = self.GetBasePoseForObjectGrasp(obj)

        if base_pose is None or grasp_config is None:
            print 'Failed to find solution'
            exit()

        r = self.base_planner.planning_env.herb.robot

        init_pose = self.base_planner.planning_env.herb.GetCurrentConfiguration()
        # Now plan to the base pose
        start_pose = numpy.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())
        base_plan = self.base_planner.Plan(start_pose, base_pose)
        base_traj = self.base_planner.planning_env.herb.ConvertPlanToTrajectory(base_plan)

        # For visualization, move robot to pickup location before planning
        theta = base_pose[2]
        bp = numpy.array([[numpy.cos(theta), -numpy.sin(theta), 0, base_pose[0]],
                      [numpy.sin(theta),  numpy.cos(theta), 0,  base_pose[1]],
                      [0.              ,  0.              , 1,  0.  ],
                      [0.              ,  0.              , 0,  1.  ]])
        r.SetTransform(bp)

        # Now plan the arm to the grasp configuration
        start_config = numpy.array(self.arm_planner.planning_env.herb.GetCurrentConfiguration())
        arm_plan = self.arm_planner.Plan(start_config, grasp_config)
        arm_traj = self.arm_planner.planning_env.herb.ConvertPlanToTrajectory(arm_plan)

        # Move robot back to scene start
        theta=init_pose[2]
        init_pose = numpy.array([[numpy.cos(theta), -numpy.sin(theta), 0, init_pose[0]],
                      [numpy.sin(theta),  numpy.cos(theta), 0,  init_pose[1]],
                      [0.              ,  0.              , 1,  0.  ],
                      [0.              ,  0.              , 0,  1.  ]])

        # Wait for execution
        pdb.set_trace()

        print 'Executing base trajectory'
        self.base_planner.planning_env.herb.ExecuteTrajectory(base_traj)

        print 'Executing arm trajectory'
        self.arm_planner.planning_env.herb.ExecuteTrajectory(arm_traj)

        # Grasp the bottle
        task_manipulation = openravepy.interfaces.TaskManipulation(self.robot)
        task_manipulation.CloseFingers()
    

    # order the grasps - call eval grasp on each, set the 'performance' index, and sort
    def order_grasps(self, grasps):
        
        grasps_ordered = list(grasps) #you should change the order of self.grasps_ordered
        graspP = []
        ct = 0

        if len(graspP) == 0:
            return grasps
        else:
            for grasp in grasps_ordered:
              #pdb.set_trace()
                graspP[ct] = self.eval_grasp(grasp)
                ct = ct + 1
            # sort!
            order = numpy.argsort(grasps_ordered[:,graspindices.get('performance')[0]])
            order = order[::-1]
            grasps_orderd = grasps_ordered[order]
            return grasps_ordered

    def eval_grasp(self, grasp):

        contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=False)
        obj_position = self.gmodel.target.GetTransform()[0:3,3]
        # for each contact
        G = numpy.array([]) #the wrench matrix

        if numpy.shape(contacts)[0] == 0:
            return 0.0

        G = G.reshape(len(contacts[0]),0)

        for c in contacts:
            pos = c[0:3] - obj_position
            direction = -c[3:] #this is already a unit vector
            m = numpy.concatenate([direction,numpy.cross(pos,direction)])
            w = numpy.vstack(m)
            G = numpy.c_[G,w]

        (U, sc, V) = numpy.linalg.svd( G, full_matrices=True)
        return min(sc)
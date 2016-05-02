import logging, numpy, openravepy
import pdb
import time

class GraspPlanner(object):

    def __init__(self, robot, base_planner, arm_planner):
        self.robot = robot
        self.base_planner = base_planner
        self.arm_planner = arm_planner

            
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
        base_pose = numpy.array([[numpy.cos(theta), -numpy.sin(theta), 0, 0.0],
                      [numpy.sin(theta),  numpy.cos(theta), 0,  0],
                      [0.              ,  0.              , 1,  0.  ],
                      [0.              ,  0.              , 0,  1.  ]])

        init_pose = self.base_planner.planning_env.herb.GetCurrentConfiguration()
        theta=init_pose[2]
        init_pose = numpy.array([[numpy.cos(theta), -numpy.sin(theta), 0, init_pose[0]],
                      [numpy.sin(theta),  numpy.cos(theta), 0,  init_pose[1]],
                      [0.              ,  0.              , 1,  0.  ],
                      [0.              ,  0.              , 0,  1.  ]])

        r = self.base_planner.planning_env.herb.robot
        r.SetTransform(base_pose)

        validgrasps, validindices = gmodel.computeValidGrasps(returnnum = 7)
        
        # Todo select best grasp from valid grasp list
        validgrasp = validgrasps[0]
        pdb.set_trace()
        #gmodel.showgrasp(validgrasp)

        Tconfig = gmodel.getGlobalGraspTransform(validgrasp, collisionfree = True)

        irmodel=openravepy.databases.inversereachability.InverseReachabilityModel(robot = r)
        irmodel.load()
        densityfn,samplerfn,bounds = irmodel.computeBaseDistribution(Tconfig)

        goals = []
        numfailures = 0
        N = 1000
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
        
        r.SetTransform(goals[0][1])

        pdb.set_trace()

        grasp_config = self.arm_planner.planning_env.herb.manip.FindIKSolution(Tconfig, filteroptions=fo)

        r.SetTransform(init_pose)
        theta=0
        base_pose = goals[0][1]

        return base_pose, grasp_config

    def PlanToGrasp(self, obj):

        # Next select a pose for the base and an associated ik for the arm
        base_pose, grasp_config = self.GetBasePoseForObjectGrasp(obj)

        if base_pose is None or grasp_config is None:
            print 'Failed to find solution'
            exit()


        init_pose = self.base_planner.planning_env.herb.GetCurrentConfiguration()
        # Now plan to the base pose
        start_pose = numpy.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())
        base_plan = self.base_planner.Plan(start_pose, base_pose)
        base_traj = self.base_planner.planning_env.herb.ConvertPlanToTrajectory(base_plan)

        theta=init_pose[2]
        init_pose = numpy.array([[numpy.cos(theta), -numpy.sin(theta), 0, init_pose[0]],
                      [numpy.sin(theta),  numpy.cos(theta), 0,  init_pose[1]],
                      [0.              ,  0.              , 1,  0.  ],
                      [0.              ,  0.              , 0,  1.  ]])
        r = self.base_planner.planning_env.herb.robot
        r.SetTransform(init_pose)

        pdb.set_trace()
        print 'Executing base trajectory'
        self.base_planner.planning_env.herb.ExecuteTrajectory(base_traj)

        # Now plan the arm to the grasp configuration
        start_config = numpy.array(self.arm_planner.planning_env.herb.GetCurrentConfiguration())
        arm_plan = self.arm_planner.Plan(start_config, grasp_config)
        arm_traj = self.arm_planner.planning_env.herb.ConvertPlanToTrajectory(arm_plan)

        print 'Executing arm trajectory'
        self.arm_planner.planning_env.herb.ExecuteTrajectory(arm_traj)

        # Grasp the bottle
        task_manipulation = openravepy.interfaces.TaskManipulation(self.robot)
        task_manipulation.CloseFingers()
    

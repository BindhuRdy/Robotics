import numpy
import pylab as pl

class SimpleEnvironment(object):
    
    def __init__(self, herb):
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5.], [5., 5.]]

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.0], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        # goal sampling probability
        self.p = 0.0

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        
    def GenerateRandomConfiguration(self):
        config = [0] * 2;
        lower_limits, upper_limits = self.boundary_limits
        #
        # TODO: Generate and return a random configuration
        #
        for i in range(len(config)):
            config[i] = lower_limits[i]+(upper_limits[i]-lower_limits[i])*numpy.random.random()
            #print "%r < %r < %r" % (lower_limits[i], config[i], upper_limits[i])

        return numpy.array(config)

    def ComputeDistance(self, start_config, end_config):
        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        #
        x = numpy.zeros(7)
        for i in range(len(start_config)):
            x[i] = (start_config[i] - end_config[i])

        #return numpy.sqrt(x)
        return (numpy.linalg.norm(x))

    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #
        numsteps = 100.0
        dimensions = 7
        steps = numpy.zeros((dimensions, int(numsteps)))
        

        for i in range(dimensions):
            #diff1 = start_config[i] - end_config[i]
            #diff2 = end_config[i] - start_config[i]
            #if diff1 < diff2:
            #    diff = diff1
            #else:
            #    diff = diff2
            diff = end_config[i] - start_config[i]
            this_dim_steps = range(1, int(numsteps)+1)
            steps[i] = [x * (diff / numsteps) + start_config[i] for x in this_dim_steps]
        #print steps
        #steps = numpy.ndarray(steps)
        joints = self.robot.GetActiveDOFIndices()
        original_values = self.robot.GetDOFValues()
        # steps[i][j], refers to the i-th dimension and the j-th step
        env = self.robot.GetEnv()
        for j in range(int(numsteps)):
             # Transform robot to new position
            #import IPython
            #IPython.embed()
            values = steps[:,j]
            self.robot.SetActiveDOFValues(values)
            with self.robot.GetEnv():
                self.robot.SetActiveDOFValues(values)
            #print values    
            for b in self.robot.GetEnv().GetBodies():
                #if b.GetName() == self.robot.GetName():
                #    continue
                 #print "Body Transform: %r" % b.GetTransform()
                 # Check each body for collision with the robot
                q = self.robot.GetEnv().CheckCollision(b, self.robot)
                #print q
                if q:
                    #print 'collision!'
                    self.robot.SetActiveDOFValues(original_values)
                    with self.robot.GetEnv():
                        self.robot.SetActiveDOFValues(original_values)
                    return None
        self.robot.SetActiveDOFValues(values)       
        with self.robot.GetEnv():
            self.robot.SetActiveDOFValues(values)       
        return end_config

    def ShortenPath(self, path, timeout=5.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #
        now = 0
        while now < timeout:
            now = copy.deepcopy(time.clock() - now)        
            g = copy.deepcopy(path[-1])     
            i = 0 
            while path[i+1].all() != g.all():
                if len(path) > 3:
                    if self.Extend(path[i],path[i+2]) != None:
                        del path[i+1]
                    i = i + 1
                else:
                    break
                if len(path) - i < 2:
                    break    
            now = copy.deepcopy(time.clock() - now)

        return path


    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()


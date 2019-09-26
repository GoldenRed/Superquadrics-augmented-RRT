
import numpy as np

class RRT:
    def __init__(self, actuatorConfigs, goal, countBiasNum=100):
        self.actuatorConfigs = actuatorConfigs #should look something like: [[0,0,1], [1,0,0], [0,1,0], [0,0,1], [1,0,0]] for an arm with 6 revolute links.
        self.goal = goal
        self.count = 0
        self.countBiasNum = countBiasNum
        self.numLinks = len(actuatorConfigs) #ie how many links are there in the arm
        self.nodes = []

        self.maxDistDiff = np.pi/18 #10 degrees
        #TODO:
        #self.path = hur ska jag lagra paths?

    def generateRandom(self):
        '''
        actuatorConfigs contains actuator limitation info, i.e. around which axis (x, y or z or in terms of phi, theta, psi) the revolute actuator for each link can rotate.can
        The len(actuatorConfigs) informs the amount of links we have in the arm
        example actuatorConfigs: [[0,0,1], [1,0,0], [0,1,0]] 
        It might seem strange but it is a compromise to make it work with the SQ library.
        '''
        tau = np.pi*2 #Note that the angles will be nonnegative due to this function
        randArmOrder = []
        if self.count != self.countBiasNum:
            for i in range(self.numLinks):
                randArmOrder.append([actuatorConfigs[i][0]*tau*np.random.rand(), actuatorConfigs[i][1]*tau*np.random.rand(), actuatorConfigs[i][2]*tau*np.random.rand()])
        else:
            randArmOrder = self.goal

        self.count = self.count + 1
        return randArmOrder

    def dist(self, node1, node2):
    #This function is critical as it is important in how the algorithm extends
    
        ##what does distance mean in this case even?
        #euclidean distance between angles? Does that even make sense? 
        #Luckily the angles are all non-negative
        distance = 0
        for i in range(self.numLinks):
            tempDist = (node1[i][0]-node2[i][0]) + (node1[i][1]-node2[i][1]) + (node1[i][2]-node2[i][2])
            distance = distance + tempDist**2
            
        #note that this will produce euclidean distances between angles for revolute joints, but if the
        #arm configuration allows for a different type of joint (e.g. [1,1,1], shoulder joint) it will take the
        #manhattan distance I suppose between those angles (if that is useful?)
        print(distance, np.sqrt(distance))
        return np.sqrt(distance)

    def findNearest(self, node_random):
        '''
        Will return index of closest node
        '''
        bestIndex = 0
        bestDistance = self.dist(self.nodes[0], node_random) #use the initial point as reference
        for index in range(len(self.nodes)):
            distance = self.dist(self.nodes[index], node_random)
            if distance < bestDistance:
                bestIndex = index
                bestDistance = distance
        return bestIndex 

    def initNode(self):
        '''initialises with a starting node.starting
        each node should be a self.numLinksx3 matrix, where each row should have be either [1, 0, 0], [0, 1, 0], or [0, 0, 1] (but doesn't have to be)  '''
        node = self.numLinks*[[0,0,0]]
        return node


    def checkLineSegment(self, node1, node2):
        #does it have to check the whole line or just fractions of angles?
        #also, at some place it needs to check if the 
        #for ....
            # ___.armtransformorder(node1)
            #check = ___.checkcollision...
            #if check:
                #return True
        #return False
        #return TRUE or FALSE depending on the collision check
        for 
        pass

    def checkGoalReached(self, node2check):

        #either this:
        goalReachedBool = False
        check = True #
        for i in range(self.numLinks):
            d0 = node2check[i][0] - self.goal[i][0]
            d1 = node2check[i][1] - self.goal[i][1]
            d2 = node2check[i][2] - self.goal[i][2]
            check = check and ((abs(d0) < self.maxDistDiff) and (abs(d1) < self.maxDistDiff) and (abs(d2) < self.maxDistDiff))
            if not check:
                return False
        return True #This means that every check was True, i.e. every angle difference was less than self.maxDistDiff
            
        #It depends: 
            #- should the RRT find a way to a specific goal link-angle configuration?
                #Go with this algorithm:
            #- or should the RRT simply efficiently expand in space until the tip of the EE has reached its goal?
                #Do the check with "goalCheck" in Environment().checkGoal()


    def loop(self):
        #Corresponding to the "The Algorithm" subsection of the RRT section, with a slight biasing towards the goal region in self.generateRandom
        self.nodes.append(self.initNode()) #initialize with
        
        while(1): 
            node_random = self.generateRandom()
            node_nearest_index = self.findNearest(node_random)
            node_new = self.nodeNew()
            if (not self.checkLineSegment(self.nodes[node_nearest_index], node_new)):
                self.nodes.append(node_new)
                #self.path.append(...)
                if self.checkGoalReached(node_new):
                    return True
                else:
                    print('Count: ', self.count)










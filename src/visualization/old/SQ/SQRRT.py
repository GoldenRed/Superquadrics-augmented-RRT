import numpy as np
from mayavi import mlab



#Author: Yared Efrem Afework <yared94@gmail.com>
#Copyright (C) Yared Efrem Afework, Tritech Technology AB
#Requires Numpy and mayavi

#v1: allowed for basic SQ rendering and check
#v2: allowed for transformation (trans+rot) of basic SQ and collision check
#v3: allow for basic initialisation of an arm and collision check
#v4: REPLACED ROTATION MATRIX; CAN PLOT AN ARM NOW
#v5: FIXED 

#Current Todo: Figure out how to actuate the arm!

class SuperQuadric:
    def __init__(self, a1=1, a2=1, a3=1, epsilon1=1, epsilon2=1,n=80):
        #Default values give sphere superellipsoid
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3
        self.epsilon1 = epsilon1
        self.epsilon2 = epsilon2
        self.n = n

        self.loadBasicSuperEllipsoid()
        self.transMat = self.produceTransformMat([0,0,0], [0,0,0]) #this variable holds the overall transformation

    def fexp(self, base, power):
        """the signed exponential function"""
        return (np.sign(base) * (np.abs(base)**power))

    def loadBasicSuperEllipsoid(self, etamin=-np.pi/2, etamax=np.pi/2, omegamin=-np.pi, omegamax=np.pi):
        """Method that creates the parametric x,y,z values
        NOTE: SQs are surfaces in 3d space. They are created by taking the "spherical product" of vectors.
        They are reliant on not only one but two orthogonal angle. For a sphere, the first angle (eta) will create a
        semicircle in the plane (-pi/2, pi/2), while the second angle (omega) will create a semicircle in a plane 
        perpendicular to the first semicircle, according to (-pi, pi). This motivates the use of meshgrid.   """
        self.eta, self.omega = np.mgrid[etamin:etamax:complex(0,self.n), -np.pi:np.pi:complex(0,self.n)]
        self.paramx = self.a1 * (self.fexp(np.cos(self.eta), self.epsilon1)) * (self.fexp(np.cos(self.omega), self.epsilon2))
        self.paramy = self.a2 * (self.fexp(np.cos(self.eta), self.epsilon1)) * (self.fexp(np.sin(self.omega), self.epsilon2))
        self.paramz = self.a3 * (self.fexp(np.sin(self.eta), self.epsilon1))
        self.one = np.ones((self.n,self.n)) #necessary for homogenous transformation
        self.Worig = np.array([self.paramx, self.paramy, self.paramz, self.one]) 
        #local coordinate frame -> world frame directly.
        #note that self.W is a 4xnxn tensor.

    def plotSQ(self):
        """Method that plots SQ  in Mayavi"""
    # if self.loaded:
        #mlab.figure(size=(800,800), fgcolor=(0, 0, 0), bgcolor=(1, 1, 1))
        #mlab.mesh(self.paramx,self.paramy,self.paramz)
        mlab.mesh(self.W[0], self.W[1], self.W[2])
        #mlab.outline()
        #mlab.axes() #in mayavi/modules/axes.py change self.configure_input_data(self.axes, src.outputs[0]) to self.configure_input_data(self.axes, src.outputs[0].output)
        # mlab.title('Superquadric: ' + 'a1: ' + str(self.a1) + '; ' + 'a2: ' \
        #  + str(self.a2) + '; ' + 'a3: ' + str(self.a3) + '; ' + 'epsilon1: ' \
        #  + str(self.epsilon1) + '; ' + 'epsilon2: ' + str(self.epsilon2) + '.')
        
        # mlab.show()
    # else:
    #     self.loadBasicSuperEllipsoid()
    #     self.plotSQ()

    def checkPointCollision(self,pointx,pointy,pointz):
        """Method that checks if a point is in collission"""
        return ((pointx/self.a1)**(2/self.epsilon2) + (pointy/self.a2)**(2/self.epsilon2))**(self.epsilon2/self.epsilon1) + (pointz/self.a3)**(2/self.epsilon1)

    def produceTransformMat(self, translation = [0, 0, 0], eulerAng = [0, 0, 0]):
        """a rotation and translation expressed in RZ*RY*RX Rotation, so technically Tait Bryant
        tx = translation[0]
        ty = translation[1]
        tz = translation[2]
        phi = eulerAng[0]
        theta = eulerAng[1]
        psi = eulerAng[2]"""
        m11 = np.cos(eulerAng[0])*np.cos(eulerAng[1])
        m12 = np.cos(eulerAng[0])*np.sin(eulerAng[1])*np.sin(eulerAng[2]) - np.sin(eulerAng[0])*np.cos(eulerAng[2])
        m13 = np.cos(eulerAng[0])*np.sin(eulerAng[1])*np.cos(eulerAng[2]) + np.sin(eulerAng[0])*np.sin(eulerAng[2])
        m14 = translation[0]

        m21 = np.sin(eulerAng[0])*np.cos(eulerAng[1])
        m22 = np.sin(eulerAng[0])*np.sin(eulerAng[1])*np.sin(eulerAng[2]) + np.cos(eulerAng[0])*np.cos(eulerAng[2])
        m23 = np.sin(eulerAng[0])*np.sin(eulerAng[1])*np.cos(eulerAng[2]) - np.cos(eulerAng[0])*np.sin(eulerAng[2])
        m24 = translation[1]

        m31 = -np.sin(eulerAng[1])
        m32 = np.cos(eulerAng[1])*np.sin(eulerAng[2])
        m33 = np.cos(eulerAng[1])*np.cos(eulerAng[2])
        m34 = translation[2]


        transMat = np.array([[m11, m12, m13, m14], [m21, m22, m23, m24], [m31, m32, m33, m34], [0, 0, 0, 1]])
        return transMat

    def transform(self, translation = [0, 0, 0], rotation = [0, 0, 0], incomingOutsideTransMat=False, outsideTransMat=None):
        """Transforms all of the x,y,z points  homogenously. Keeps track of previous transformations through self.transMat
        outsideTransMat is used to keep the amount of transformations low. It is used for getting the link to where the previous 
        links are located, but then after that the link can be shifted by itself"""

        if incomingOutsideTransMat == False:
            tempTransMat = self.produceTransformMat(translation, rotation)
            self.transMat = np.matmul(tempTransMat,self.transMat) #Update self.transMat
        else:
            self.transMat = np.matmul(outsideTransMat,self.transMat)

        Wnew = np.tensordot(self.transMat, self.Worig, axes=((1),(0))) #this is equivalent to the commented nested-for loop below:

        #L,J,K = self.Worig.shape
        # Wnew1 = np.zeros((L,J,K))
        # for j in range(J):
        #     for k in range(K):
        #         for l in range(L):
        #             Wnew1[l,j,k] = self.transMat[l,0]*self.W[0,j,k] + self.transMat[l,1]*self.W[1,j,k] + self.transMat[l,2]*self.W[2,j,k] + self.transMat[l,3]*self.W[3,j,k]
        self.W = Wnew
        #print("Transformed.")

    def reset(self):
        self.transMat = self.produceTransformMat([0,0,0], [0,0,0])


    def checkPointCollisionPostTransformation(self,pointx,pointy,pointz):
        """Method that checks if a point is in collission"""
        nx = self.transMat[0][0]
        ny = self.transMat[1][0]
        nz = self.transMat[2][0]

        ox = self.transMat[0][1]
        oy = self.transMat[1][1]
        oz = self.transMat[2][1]

        ax = self.transMat[0][2]
        ay = self.transMat[1][2]
        az = self.transMat[2][2]

        px = self.transMat[0][3] 
        py = self.transMat[1][3]
        pz = self.transMat[2][3]

        expression1 = nx*pointx + ny*pointy + nz*pointz - px*nx - py*ny - pz*nz
        expression2 = ox*pointx + oy*pointy + oz*pointz - px*ox - py*oy - pz*oz
        expression3 = ax*pointx + ay*pointy + az*pointz - px*ax - py*ay - pz*az
        return ((expression1/self.a1)**(2/self.epsilon2) + (expression2/self.a2)**(2/self.epsilon2))**(self.epsilon2/self.epsilon1) + (expression3/self.a3)**(2/self.epsilon1)


class Arm:
    def __init__(self, base=SuperQuadric(1,1,1,0.2,0.2)):
        self.links = []
        self.configuration = []
        self.addLink(base)
        self.extras = []

    def actuateArm(self, order=None):
        """Once the arm has been initialised we only want to be able to rotate the joints"""
        #reset the links from their previous configuration (e.g., )
        if order==None:
            order=len(self.links)*[[0,np.pi/6,0]] #for testing

        #Reset the links:
        for link in self.links:
            link.reset()

        #Actually actuate the arm by 1) getting the next link into the previous link's posiion, 2)rotate as wanted, 3) attach to the end
        for i in range(1,len(self.links)):
            link_prev = self.links[i-1]
            link_next = self.links[i]

            #Set current link to where previous link is:
            link_next.transform(incomingOutsideTransMat=True, outsideTransMat=link_prev.transMat)
            #Rotate it as wished:
            link_next.transform(incomingOutsideTransMat=False, rotation=order[i])
            #Translate it into position
            link_next.transform(incomingOutsideTransMat=False, translation=[(link_prev.W[0][-1][link_prev.n//2] - link_next.W[0][0][link_next.n//2]), (link_prev.W[1][-1][link_prev.n//2] - link_next.W[1][0][link_next.n//2]), (link_prev.W[2][-1][link_prev.n//2] - link_next.W[2][0][link_next.n//2])])


    def addLink(self, primitive, translation=[0,0,0], rotation=[0,0,0]):
        """Don't use this...
        If you want to directly add the SQ primitive, translation and rotation.
        Is called by default for the original base piece"""
        primitive.loadBasicSuperEllipsoid()
        primitive.transform(translation,rotation)
        self.configuration.append([translation, rotation])
        self.links.append(primitive)

    def newStandardLink(self, xdimlength=1, ydimlength=1, zdimlength=10, eps1shape=0.2, eps2shape=0.2):
        """Will add "standard" link of a certain height"""
        primitive = SuperQuadric(xdimlength,ydimlength,zdimlength, eps1shape, eps2shape) #Standard link
        #then shift it to the right height:
        self.provideInitialTranslation(primitive,self.links[-1])
        #add it to the list of links
        self.links.append(primitive)

    def provideInitialTranslation(self, primitive, prevlink):
        """The idea is that the arm begins with all links pointing STRAIGHT up in the z-axis"""
        
        #get the current link to the previous links position:
        primitive.transform(incomingOutsideTransMat=True, outsideTransMat=prevlink.transMat)

        #current link needs to shifted by previous link's a3 value and its own a3 value to stop being inside it
        shift = primitive.a3 + prevlink.a3
        primitive.transform(translation=[0,0,shift], rotation=[0,0,0], incomingOutsideTransMat=False)


    def checkCollision(self, pointx, pointy, pointz):
        F_list = []
        for link in self.links:
            F_list.append(link.checkPointCollisionPostTransformation(pointx, pointy, pointz))
        return F_list

    def checkCollisionBoolean(self, pointx, pointy, pointz):
        #F_list = []
        for i in range(len(self.links)):
            #print(i, self.links[i].checkPointCollisionPostTransformation(pointx, pointy, pointz))
            if 1 > self.links[i].checkPointCollisionPostTransformation(pointx, pointy, pointz):
                print('COLLISION!!!!!!!!!!!!!') #At: ', pointx, pointy, pointz, '; with link: ' i)
        return False


    def plotArm(self):
        """Method that plots SQ  in Mayavi"""
        #If you get error with the axes, do the following replacement:
        #in mayavi/modules/axes.py change self.configure_input_data(self.axes, src.outputs[0]) to self.configure_input_data(self.axes, src.outputs[0].output)
        #and to get rid of the ugly error messages, in mayavi/tools/decorations.py change axes.axes.ranges = axes.module_manager.source.outputs[0].bounds to
        """src = axes.module_manager.source
        data = src.outputs[0] if not hasattr(src.outputs[0], 'output') else src.outputs[0].output
        axes.axes.ranges = data.bounds"""
        #mlab.figure(size=(800,800), fgcolor=(0, 0, 0), bgcolor=(1, 1, 1))
        for primitive in self.links:
            mlab.mesh(primitive.W[0], primitive.W[1], primitive.W[2])
            #if primitive == self.links[0]:
                #give the base SQ axes and outline
                #mlab.axes()
                #mlab.outline()

        #mlab.axes()
        #mlab.outline()




class Environment:
    def __init__(self):
        self.Arms = []
        self.Obstacles = []
        self.goal = None
    def addArm(self, arm):
        self.Arms.append(arm)
    def addObstacle(self, obstacle=None, dims=[1,1,1,1,1], transform=[[0,0,0], [0,0,0]]):
        if obstacle == None:
            obstacle = SuperQuadric(a1=dims[0], a2=dims[1], a3=dims[2], epsilon1=dims[3], epsilon2=dims[4])
            obstacle.transform(translation=transform[0], rotation=transform[1])
        self.Obstacles.append(obstacle)
    def addGoal(self, goal=None, dims=[1,1,1,1,1], transform=[[0,0,0], [0,0,0]]):
        if goal == None:
            goal = SuperQuadric(a1=dims[0], a2=dims[1], a3=dims[2], epsilon1=dims[3], epsilon2=dims[4])
            goal.transform(translation=transform[0], rotation=transform[1])
        self.goal = [goal, transform[0]] #Contains both the SQ itself and the goal coordinates
    def actuateArm(self, number=0, order=None):
        self.Arms[number].actuateArm(order)




    def checkAllCollisions(self):
        for arm in self.Arms:
            for obstacle in self.Obstacles:
                for ii in range(obstacle.n):
                    for jj in range(obstacle.n):
                        arm.checkCollisionBoolean(obstacle.W[0][ii][jj] ,obstacle.W[1][ii][jj],obstacle.W[2][ii][jj])





    def checkGoal(self):
        if self.goal !=None:
            F = []
            for arm in self.Arms:
                EE = arm.links[-1]
                F.append(self.goal[0].checkPointCollisionPostTransformation(EE.W[0][EE.n//2][EE.n//2], EE.W[1][EE.n//2][EE.n//2], EE.W[2][EE.n//2][EE.n//2]))
            return F
        else:
            raise Exception('No Goal Set!')
    def plotEnv(self):  
        """Method that plots the whole env"""
        #If you get error with the axes, do the following replacement:
        #in mayavi/modules/axes.py change self.configure_input_data(self.axes, src.outputs[0]) to self.configure_input_data(self.axes, src.outputs[0].output)
        #and to get rid of the ugly error messages, in mayavi/tools/decorations.py change axes.axes.ranges = axes.module_manager.source.outputs[0].bounds to
        """src = axes.module_manager.source
        data = src.outputs[0] if not hasattr(src.outputs[0], 'output') else src.outputs[0].output
        axes.axes.ranges = data.bounds"""
        mlab.figure(size=(800,800), fgcolor=(0, 0, 0), bgcolor=(1, 1, 1))
        for arm in self.Arms:
            arm.plotArm()

        for obstacle in self.Obstacles:
            obstacle.plotSQ()


        if self.goal != None:
            self.goal[0].plotSQ()

        #mlab.title('World')
        mlab.show()

if __name__ == '__main__':
    import time
    
    World = Environment()

    #ADD arm:
    start = time.time()
    World.addArm(Arm())
    end = time.time()
    print("Creating arm: ", end-start)

    for i in range(3):#6):
        start = time.time()
        World.Arms[0].newStandardLink()#i*1.5, i, i*4, i, i)
        end = time.time()
        print("Adding Link: ", end-start)

    #ADD EE to arm:
    # World.Arms[0].newStandardLink(1,1,1,1,1,)

    #ACTUATE arm:
    start = time.time()
    armOrder = [[np.pi/3, 0, 0],[0, np.pi/5, 0],[0, 0, np.pi/1.5], [np.pi/2.5, 0, 0], [0, 0, np.pi/5], [np.pi/3, 0, 0], [0, 0, np.pi/3],  [0, 0, 0]]
    World.actuateArm(order=armOrder)
    end = time.time()
    print("Actuating Arm: ", end-start)


    #ADD obstacle:

    #Obstacle is convex:
    World.addObstacle(dims=[15,15,15,10,10], transform=[[30, 0, 5], [0, 0, 0]])
    # World.addObstacle(dims=[15,15,15,0.2,0.2], transform=[[0, 50, 5], [np.pi/4, 0, 0]])
    # World.addObstacle(dims=[15,15,15,0.2,0.2], transform=[[0, 0, -45], [np.pi/4, 0, 0]])


    #Obstacle is concave:
    # World.addObstacle(dims=[15,15,15,3,3], transform=[[-50, 0, 5], [np.pi/4, 0, 0]])
    # World.addObstacle(dims=[15,15,15,3,3], transform=[[0, -50, 5], [np.pi/4, 0, 0]])
    # World.addObstacle(dims=[15,15,15,3,3], transform=[[0, 0, 55], [np.pi/4, 0, 0]])


    #ADD goal:
    World.addGoal(dims=[1,1,1,1,1], transform=[[14, -2, 15], [0, 0, 0]])

    print('Check Goal: ', World.checkGoal())

    start = time.time()
    World.checkAllCollisions()
    end = time.time()
    print("Check all collisions: ", end-start)
    print(World.Arms[0].links[2].checkPointCollisionPostTransformation(14, -2, 15))



    start = time.time()
    World.plotEnv()
    end = time.time()
    print("Plotting: ", end-start)

    #print(RobotArm.links[5].W)

    # start = time.time()
    # Fs1 = RobotArm.checkCollision(9,0,12)    
    # Fs2 = RobotArm.checkCollision(58,5,-1)
    # end = time.time()
    # print("Collision Check: ", end-start)
    # print(Fs1)
    # print(Fs2)

    # Axes3D.plot(World.Obstacles[0].W[0],World.Obstacles[0].W[1],World.Obstacles[0].W[2])
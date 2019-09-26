import numpy as np
from mayavi import mlab



#Author: Yared Efrem Afework <yared94@gmail.com>
#Copyright (C) Yared Efrem Afework, Tritech Technology AB
#Requires numpy and mayavi
#Some credit goes to Casey Muratori for his video on how to optimize the GJK algorithm (GJK) using a geometric approach.

class SuperToroid:
    def __init__(self, a1=1, a2=1, a3=1, epsilon1=1, epsilon2=1,n=80):
        #Default values give sphere superellipsoid
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3
        self.epsilon1 = epsilon1
        self.epsilon2 = epsilon2
        self.n = n
        self.loadBasicSuperToroid()

    def fexp(self, base, power):
        """the signed exponential function"""
        return (np.sign(base) * (np.abs(base)**power))

    #def loadBasicSuperEllipsoid(self, etamin=-np.pi/2, etamax=np.pi/2, omegamin=-np.pi, omegamax=np.pi):
    def loadBasicSuperToroid(self, etamin=-np.pi, etamax=np.pi, omegamin=-np.pi, omegamax=np.pi):
        self.eta, self.omega = np.mgrid[etamin:etamax:complex(0,self.n), -np.pi:np.pi:complex(0,self.n)]
        self.paramx = (self.a1 + self.fexp(np.cos(self.eta), self.epsilon1))*self.fexp(np.cos(self.omega), self.epsilon2)
        self.paramy = (self.a2 + self.fexp(np.cos(self.eta), self.epsilon1))*self.fexp(np.sin(self.omega), self.epsilon2)
        self.paramz = self.a3*self.fexp(np.sin(self.eta), self.epsilon1)
        self.one = np.ones((self.n,self.n)) #necessary for homogenous transformation
        #self.Worig = np.array([self.paramx, self.paramy, self.paramz, self.one]) 
        self.W = np.array([self.paramx, self.paramy, self.paramz, self.one])
        #local coordinate frame -> world frame directly.
        #note that self.W is a 4xnxn tensor.

    def plotSQ(self):
        """Method that plots SQ  in Mayavi"""
        #mlab.figure(size=(800,800), fgcolor=(0, 0, 0), bgcolor=(1, 1, 1))
        #mlab.mesh(self.paramx,self.paramy,self.paramz)
        mlab.mesh(self.W[0], self.W[1], self.W[2])
        #mlab.outline()
        #mlab.axes() #in mayavi/modules/axes.py change self.configure_input_data(self.axes, src.outputs[0]) to self.configure_input_data(self.axes, src.outputs[0].output)
        # mlab.title('Superquadric: ' + 'a1: ' + str(self.a1) + '; ' + 'a2: ' \
        #  + str(self.a2) + '; ' + 'a3: ' + str(self.a3) + '; ' + 'epsilon1: ' \
        #  + str(self.epsilon1) + '; ' + 'epsilon2: ' + str(self.epsilon2) + '.')
        
        # mlab.show()


class GJK_Collision_Module:

    def __init__(self, link, obs):
        self.link = link
        self.obs = obs
        self.eTol = 0.1
        self.maxIter = 50

    def check_Collision(self):
        self.collision = self.GJKloop()
        return self.collision

    def GJKloop(self):
        """Based on  Casey Muratori's video"""
        iterations = 0
        S = self.SupportMapFunc() #V random in here, returns point in random direction
        W = [] #W contains all of our verteces
        W.append(S)
        V = -1*S #returns SO = O - S
        #print('S:', S, 'V:', V)

        while(1):
            iterations = iterations + 1
            if(iterations > self.maxIter):
                #print('So many iterations!', iterations, " It's probably true...")
                return True
            w = self.SupportMapFunc(V)
            if( (abs(w[0]) < self.eTol) and (abs(w[1]) < self.eTol) and (abs(w[2]) < self.eTol)):
                #print('Look at w:',w,'! It is probably the Origin')
                return True
            
            #print('New point w:',w)    
            #print('Current V:',V)
            if np.dot(w, V) <= 0:
                #print('No Intersection')
                return False
            else:
                W.append(w)
                #print('Current W:', len(W), W)
                W, V = self.DoSimplex(W,V)
                if len(V) == 0:
                    return True

    def DoSimplex(self, W, V):
        if len(W) == 0:
            print('Impossible n=0 case')
            return W, V

        elif len(W) == 1:
            print('Impossible n=1 case')
            return W, V

        elif len(W) == 2:
            #print('n=2 case')
            return self.TwoPoints(W,V)

        elif len(W) == 3:
            #print('n=3 case')
            return self.ThreePoints(W,V)

        elif len(W) == 4:
            #print('n=4 case')
            return self.FourPoints(W,V)

        else:
            print('n>4 case')
            return

    def TwoPoints(self, W,V):
        #Line Scenario
        #B = W[0] prev point
        #A = W[1] new point
        A = W[-1] #last added point
        AO = -1*A #direction to origin from A
        B = W[-2] #previous point
        AB = B-A #line from new point to old
        #There is no BO because we have previously ruled out that region (the region closest to B)
        if np.dot(AB, AO)>0:
            #print('TwoPoints, O is in AB-region')
            newV = np.cross(np.cross(AB,AO),AB)
            return W, newV
        else:
            #print('TwoPoints, O is in A-region')
            newW = [A]
            newV = AO
            return newW, newV

    #QUESTION: If we return newW, going by Casey's notation should I return it as [A, B, C] or [C, B, A]??
        
    def ThreePoints(self, W,V):
        #Triangle Scenario
        A = W[-1]
        B = W[-2]
        C = W[-3]
        AO = -1*A #Direction to origin from new point A
        AB = B - A #One edge of triangle
        AC = C - A #One edge of triangle
        ABC = np.cross(AB, AC) #Vector pointing downward out of triangle
        #There is no BC, BO or CO because we have previously ruled out those areas (BC edge; B and C points)

        if np.dot(np.cross(ABC,AC), AO) > 0: #We check if origo is perpendicular to AC-edge
            if np.dot(AC, AO) > 0: #We check if it is specficically in AC-edge region            
                #Case 1
                #newW = [A, C]
                #print('ThreePoints, O is in AC-region')
                newW = [C, A]
                newV = np.cross( np.cross(AC,AO), AC)
                return newW, newV
            else:
                #Is it in AB-edge or A region?
                return self.STARFUNCTION(A, B, AO, AB)

        else: #It is not perpendicular to AC-edge region
            if np.dot(np.cross(AB,ABC),AO) > 0: #We check if it is perpendicular to AB-region
                return self.STARFUNCTION(A, B, AO, AB)
            else:
                #It must be above or below the triangle (we have ruled out BC, B, C regions already)
                if np.dot(ABC, AO) > 0:
                    #The origin is below the triangle
                    #Case 2
                    #newW = [A, B, C]
                    #print('ThreePoints, O is below ABC')
                    #newW = [C, B, A]
                    #flip the triangle:
                    newW = [B, C, A]
                    newV = ABC
                    return newW, newV
                else:
                    #The origin is above the triangle:
                    #Case 3
                    #newW = [A, C, B] #we flip the triangle
                    #print('ThreePoints, O is above ABC')
                    newW = [C, B, A]
                    newV = -1*ABC
                    return newW, newV


    def STARFUNCTION(self, A, B, AO, AB):
        """because Casey Muratori used a star in his video for this """
        if np.dot(AB, AO) > 0: #is the point in AB-region?
            #The point is in AB region
            #Case 4
            #newW = [A,B]
            #print('ThreePoints, O is in AB-region')
            newW = [B,A]
            newV = np.cross(np.cross(AB,AO), AB)
            return newW, newV
        else: 
            #The point is in the A region
            #Case 5
            #print('ThreePoints, O is in A-region')
            newW = [A]
            newV = AO
            return newW, newV





    def FourPoints(self, W,V):
        #TETRAHEDRAL SCENARIO
        #But we're basically looking at the 3 (out of 4) faces of the tetrahedron separately (since we already check that). The code for checking the A-point region as well as some of the 

        A = W[-1]
        B = W[-2]
        C = W[-3]   
        D = W[-4]

        #We don't check the DCB triangle

        CBA_BOOL = self.ThreePoints_TetraCase([C, B, A], V)
        BDA_BOOL = self.ThreePoints_TetraCase([B, D, A], V)
        DCA_BOOL = self.ThreePoints_TetraCase([D, C, A], V)

        #111
        if (CBA_BOOL and BDA_BOOL and DCA_BOOL) == 1: #111
            #print('Origin ENCAPSULATED!!!!!!!!!!!!!!')
            return W, []


    #These three tell us to kick out one point:
        
        elif(CBA_BOOL and (not BDA_BOOL) and (not DCA_BOOL)) == 1:#100
            #print('Kick out D')
            newW = [C, B, A]
            #search in the opposite direction of D?
            newV = -1*D
            return newW, newV

        elif((not CBA_BOOL) and (not BDA_BOOL) and DCA_BOOL) == 1: #001
            #print('Kick out B')
            newW = [D, C, A]
            #search in the opposite direction of B?
            newV = -1*B
            return newW, newV

        elif((not CBA_BOOL) and BDA_BOOL and (not DCA_BOOL)) == 1: #010
            #print('Kick out C')
            newW = [D, B, A]
            #search in the opposite direction of C?
            newV = -1*C
            return newW, newV


    #This one tells us to kick out B,C,D and start over 
        elif((not CBA_BOOL) and (not BDA_BOOL) and (not DCA_BOOL)) == 1: #000
            #print('Kick out B, C, D!')
            #print('Check Out: NOT_DCB_BOOL:', (not ThreePoints_TetraCase([D, C, B], V)))
            newW = [A]
            newV = -1*A
            return newW, newV
        

        
    #These three tell us to kick out all but one:
        #(But they could also lead to the algorithm looping over and over)
        elif((not CBA_BOOL) and (BDA_BOOL) and (DCA_BOOL)) == 1: #011
            #origin is probably in the CBA triangle or incredibly close, so we kick out all but D 
            newW = [D]
            newV = -1*D
            return newW, newV
        
        elif((CBA_BOOL) and (not BDA_BOOL) and (DCA_BOOL)) == 1: #101
            #origin is probably in the BDA triangle or incredibly close, so we kick out all but C 
            newW = [C]
            newV = -1*C
            return newW, newV
        elif((CBA_BOOL) and (BDA_BOOL) and (not DCA_BOOL)) == 1: #110
            #origin is probably in the DCA triangle or incredibly close, so we kick out all but B 
            newW = [B]
            newV = -1*B
            return newW, newV



        else:
            print('CBA_BOOL:', CBA_BOOL, 'BDA_BOOL:', BDA_BOOL, 'DCA_BOOL:', DCA_BOOL)
            #This case is not supposed to be happen?
            print('WTF? This is not supposed to happen...')
            print('Check Out: NOT_DCB_BOOL:', (not self.ThreePoints_TetraCase([D, C, B], V)))


    def ThreePoints_TetraCase(self, W,V):
        #Basically we are ONLY interested in knowing if the origin is in the inside-direction region or any other direction of the triangle
        A = W[-1]
        B = W[-2]
        C = W[-3]
        AO = -1*A #Direction to origin from new point A
        AB = B - A #One edge of triangle
        AC = C - A #One edge of triangle
        ABC = np.cross(AB, AC) #Vector pointing downward out of triangle
        #There is no BC, BO or CO because we have previously ruled out those areas

        if np.dot(np.cross(ABC,AC), AO) > 0: #
            if np.dot(AC, AO) > 0:
                #Case 1
                newW = [C, A]
                newV = np.cross(np.cross(AC,AO), AC)
                return False #newW, newV
            else:
                return False #STARFUNCTION_TetraCase(A, B, AO, AB)

        else:
            if np.dot(np.cross(AB,ABC),AO) > 0:
                return False #STARFUNCTION_TetraCAse(A, B, AO, AB)
            else:
                if np.dot(ABC, AO) > 0:
                    #Case 2
                    #The origin is below
                    newW = [C, B, A]
                    newV = ABC
                    return True #newW, newV
                else:
                    #Case 3
                    #The origin is above
                    newW = [B, C, A]
                    newV = -1*ABC
                    return False #newW, newV

    def SupportMapFunc(self, V = np.random.rand(3,1)):
        S_Link = self.FurthestPointInDirection(self.link, V)
        S_Obs = self.FurthestPointInDirection(self.obs, V, reverse=True)

        return S_Link - S_Obs

    def FurthestPointInDirection(self, convexSet, V, reverse=False):
        if reverse==True: 
            #points = -1*convexSet.W[0].reshape(convexSet.n**2), -1*convexSet.W[1].reshape(convexSet.n**2), -1*convexSet.W[2].reshape(convexSet.n**2) #rearrangaement perhaps
            V = -1*V
        points = convexSet.W[0].reshape(convexSet.n**2), convexSet.W[1].reshape(convexSet.n**2), convexSet.W[2].reshape(convexSet.n**2) #rearrangaement perhaps

        bestIndex = 0
        bestDotP = np.dot((points[0][bestIndex], points[1][bestIndex], points[2][bestIndex]), V)
        for i in range(convexSet.n**2):
            d = np.dot([points[0][i], points[1][i], points[2][i]], V)
            if d>bestDotP:
                bestIndex = i
                bestDotP = d
        #print('Best Index:', bestIndex)
        #print('Best Dot Product:', bestDotP)
        furthestPoint = np.array([points[0][bestIndex], points[1][bestIndex], points[2][bestIndex]])
        return furthestPoint


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
            if 1 > abs(self.links[i].checkPointCollisionPostTransformation(pointx, pointy, pointz)):
                # print('COLLISION!!!!!!!!!!!!!') #At: ', pointx, pointy, pointz, '; with link: ' i)
                #print('SQ Collision, link:', i)
                return True
        return False


    def plotArm(self, color_boolean=False, colorVec=None):
        """Method that plots SQ  in Mayavi"""
        #If you get error with the axes, do the following replacement:
        #in mayavi/modules/axes.py change self.configure_input_data(self.axes, src.outputs[0]) to self.configure_input_data(self.axes, src.outputs[0].output)
        #and to get rid of the ugly error messages, in mayavi/tools/decorations.py change axes.axes.ranges = axes.module_manager.source.outputs[0].bounds to
        """src = axes.module_manager.source
        data = src.outputs[0] if not hasattr(src.outputs[0], 'output') else src.outputs[0].output
        axes.axes.ranges = data.bounds"""
        #mlab.figure(size=(800,800), fgcolor=(0, 0, 0), bgcolor=(1, 1, 1))
        if color_boolean:
            for primitive in self.links:
                mlab.mesh(primitive.W[0], primitive.W[1], primitive.W[2], color=colorVec)
        else:
            for primitive in self.links:
                mlab.mesh(primitive.W[0], primitive.W[1], primitive.W[2])
                #if primitive == self.links[0]:
                    #give the base SQ axes and outline
                    #mlab.axes()
                    #mlab.outline()

        #mlab.axes()
        #mlab.outline()

class RRT:
    def __init__(self, WORLDENV, actuatorConfigs, goal, start, countBiasNum=50, ):
        self.world = WORLDENV
        self.actuatorConfigs = actuatorConfigs #should look something like: [[0,0,1], [1,0,0], [0,1,0], [0,0,1], [1,0,0]] for an arm with 6 revolute links.
        self.goal = goal
        self.count = 0# to make it 0 "indexed"
        self.countBiasNum = countBiasNum
        self.numLinks = len(actuatorConfigs) #ie how many links are there in the arm
        self.nodes = []
        self.start = start
        self.maxDistDiff = np.pi/18 #maximum amount allowed to be diffing from the goal
        self.absDistDiff = 5 #creates a box goal area
        self.maxJump = 0.5
        #TODO:
        self.edges = [] #collection of collision-checked lines (done with 10 steps)
        self.OK_path = [] #will (hopefully) contain the indeces of the vertices route from Init to Goal

        self.first = True #boolean for recursive path find, i.e. makes self.OK_PATH
        self.setGoalXYZ()

    def setGoalXYZ(self):

        self.world.actuateArm(order=self.goal)
        self.goalX = World.Arms[0].links[-1].W[0][-1][-1]
        self.goalY = World.Arms[0].links[-1].W[1][-1][-1]
        self.goalZ = World.Arms[0].links[-1].W[2][-1][-1]



    def getAnglesOut(self):
        if not len(self.OK_path)==0:
            self.OK_path_Orders = []
            for val in self.OK_path:
                self.OK_path_Orders.append(self.nodes[val])
        else:
            print('No OK path available.')
    
    def recuPathFind(self, prevLink=None):
        if self.first == True:
            self.OK_path = []
            self.OK_path.append(self.edges[-1][1])
            prevLink = self.edges[-1]
            self.first = False
            return self.recuPathFind(prevLink)
        else:
            print('Previous Link:', prevLink)
            self.OK_path.append(prevLink[0])
            if(prevLink[0]) == 0:
                self.first = True #I.e, we are done and reset the state.
                return
            prevLink = self.edges[prevLink[0]-1]
            return self.recuPathFind(prevLink)

    def generateRandom(self):
        '''
        actuatorConfigs contains actuator limitation info, i.e. around which axis (x, y or z or in terms of phi, theta, psi) the revolute actuator for each link can rotate.can
        The len(actuatorConfigs) informs the amount of links we have in the arm
        example actuatorConfigs: [[0,0,1], [1,0,0], [0,1,0]] 
        It might seem strange but it is a compromise to make it work with the SQ library.
        '''
        tau = np.pi*2 #Note that the angles will be nonnegative due to this function
        randArmOrder = []
        if (self.count%self.countBiasNum) != 0:
            for i in range(self.numLinks):
                #randArmOrder.append([actuatorConfigs[i][0]*tau*np.random.rand(), actuatorConfigs[i][1]*tau*np.random.rand(), actuatorConfigs[i][2]*tau*np.random.rand()])
                randArmOrder.append([actuatorConfigs[i][0]*tau*(np.random.rand() - 1), actuatorConfigs[i][1]*tau*(np.random.rand()- 1), actuatorConfigs[i][2]*tau*(np.random.rand() - 1)])
        else:
            randArmOrder = self.goal
            #print('biased towards goal!')

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
        #print(distance, np.sqrt(distance))
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
        node = self.start
        return node


    def checkLineSegment(self, node1, node2):
        #does it have to check the whole line or just fractions of angles?
        node1 = np.array(node1)
        node2 = np.array(node2)
        for i in range(1, 10): #It jumps in tenths, which might or might not be catastrophically bad
            armOrder = node1*(1-0.1*i) + 0.1*i*node2
            self.world.actuateArm(order=armOrder.tolist()) #actuateArm takes in normal python list, not np.array (I think...)
            #if self.world.checkAllCollisionsSQ(): #ie, true collision 
            #if self.world.checkAllCollisionsGJK(): #ie, true collision #checkAllCollisionsReverseSQ
            if self.world.checkAllCollisionsReverseSQ():
                return False #i.e, failed to pass test checkLineSegment
        return True #passed test, 

    def checkGoalReached(self, node2check):
    #It depends: 
        #- should the RRT find a way to a specific goal link-angle configuration?
            #Go with this algorithm:
        #- or should the RRT simply efficiently expand in space until the tip of the EE has reached its goal?
            #Do the check with "goalCheck" in Environment().checkGoal()


        self.world.actuateArm(order=node2check)
        nodex = self.world.Arms[0].links[-1].W[0][-1][-1]
        nodey = self.world.Arms[0].links[-1].W[1][-1][-1]
        nodez = self.world.Arms[0].links[-1].W[2][-1][-1]

        if (abs(nodex - self.goalX)<self.absDistDiff) and (abs(nodey - self.goalY)<self.absDistDiff) and (abs(nodez - self.goalZ)<self.absDistDiff):
            return True
        else:
            return False
        #or this:

        # goalReachedBool = False
        # check = True #
        # for i in range(self.numLinks):
        #     d0 = node2check[i][0] - self.goal[i][0]
        #     d1 = node2check[i][1] - self.goal[i][1]
        #     d2 = node2check[i][2] - self.goal[i][2]
        #     check = check and ((abs(d0) < self.maxDistDiff) and (abs(d1) < self.maxDistDiff) and (abs(d2) < self.maxDistDiff))
        #     if not check:
        #         #print('check false:', check)
        #         return False
        # print('check true:', check)
        # return True #This means that every check was True, i.e. every angle difference was less than self.maxDistDiff

    def generateNodeNew(self, startNode, goalNode):
        startNode = np.array(startNode)
        goalNode = np.array(goalNode)
        nodeNew = startNode + goalNode*self.maxJump
        return nodeNew.tolist()

    def loop(self):
        #Corresponding to the "The Algorithm" subsection of the RRT section, with a slight biasing towards the goal region in self.generateRandom
        self.nodes.append(self.initNode()) #initialize with
        
        while(1): 
            node_random = self.generateRandom() #generate random new node
            node_nearest_index = self.findNearest(node_random) #find node_nearest in the graph
            node_new = self.generateNodeNew(self.nodes[node_nearest_index], node_random) #generate a node_new located from node_nearest in the direction of node_random a specific distance
            if self.checkLineSegment(self.nodes[node_nearest_index], node_new): #Check the line between node_nearest and node_new; is the line clear of collisions? (checked over 10 intervals)
                self.nodes.append(node_new) #add this node_new to the graph since we have checked that it is OK to go to it
                self.edges.append([ node_nearest_index, len(self.nodes)-1]) #save the edges, as a list of lists [prev_node, next_node]
                if self.checkGoalReached(node_new): #check if the goal has been reached
                    print('GOAL REACHED!!!')
                    self.recuPathFind() #We need to check the progression from 
                    print(self.OK_path) 
                    return True
            #print('Count: ', self.count)


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




    def checkAllCollisionsSQ(self):
        #Arms Implicit, Obstacles Explicit
        c_OSQ = 0
        for arm in self.Arms:
            for obstacle in self.Obstacles:
                c_OSQ = c_OSQ +1
                #print('c_OSQ:', c_OSQ)

                for ii in range(obstacle.n):
                    for jj in range(obstacle.n):
                        if arm.checkCollisionBoolean(obstacle.W[0][ii][jj], obstacle.W[1][ii][jj], obstacle.W[2][ii][jj]):
                            return True
                        #checks all points in the obstacle 

    def checkAllCollisionsGJK(self):

        for arm in self.Arms:
            for obstacle in self.Obstacles:
                if (obstacle.epsilon1>2) or (obstacle.epsilon2>2): 
                    raise Exception('Object NOT Convex!')
                for ii  in range(len(arm.links)):
                    GJKChecker = GJK_Collision_Module(arm.links[ii], obstacle)
                    if GJKChecker.check_Collision():
                        #print('GJK COLLISION DETECTED, LINK: ', ii)
                        return True

    def checkAllCollisionsReverseSQ(self):
        #Arms Explicit, Obstacles Implicit
        c_RSQ = 0
        for obstacle in self.Obstacles:
            for arm in self.Arms:
                for link in arm.links:
                    c_RSQ = c_RSQ +1
                    #print('c_RSQ:',c_RSQ)
                    for ii in range(link.n):
                        for jj in range(link.n):
                            #print(link, abs(obstacle.checkPointCollisionPostTransformation(link.W[0][ii][jj], link.W[1][ii][jj], link.W[2][ii][jj])))
                            if (1 > abs(obstacle.checkPointCollisionPostTransformation(link.W[0][ii][jj], link.W[1][ii][jj], link.W[2][ii][jj]))):
                                #print('RevSQ Collision!!!')
                                return True


    def checkGoal(self):
        if self.goal !=None:
            F = []
            for arm in self.Arms:
                EE = arm.links[-1]
                F.append(self.goal[0].checkPointCollisionPostTransformation(EE.W[0][EE.n//2][EE.n//2], EE.W[1][EE.n//2][EE.n//2], EE.W[2][EE.n//2][EE.n//2]))
            return False
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

    for i in range(6):
        start = time.time()
        World.Arms[0].newStandardLink(eps1shape=0.7, eps2shape=1)#i*1.5, i, i*4, i, i)
        end = time.time()
        print("Adding Link: ", end-start)

    #ADD spherical EE to arm:
    # World.Arms[0].newStandardLink(1,1,1,1,1)

    # #ACTUATE arm:
    # start = time.time()
    # armOrder = [[np.pi/3, 0, 0],[0, np.pi/5, 0],[0, 0, np.pi/1.5], [np.pi/2.5, 0, 0], [0, 0, np.pi/5], [np.pi/3, 0, 0], [0, 0, np.pi/3],  [0, 0, 0], [np.pi/3, 0, 0],[0, np.pi/5, 0],[0, 0, np.pi/1.5], [np.pi/2.5, 0, 0], [0, 0, np.pi/5], [np.pi/3, 0, 0], [0, 0, np.pi/3],  [0, 0, 0]]
    # World.actuateArm(order=armOrder)
    # end = time.time()
    # print("Actuating Arm: ", end-start)


    #ADD obstacle:

    #Obstacle is convex:    
    World.addObstacle(dims=[5,5,5,1, 1], transform=[[50, 0, 0], [0, 0, 0]])

    #Obstacle is concave:
    # World.addObstacle(dims=[15,15,15,3,3], transform=[[-50, 0, 5], [np.pi/4, 0, 0]])
    # World.addObstacle(dims=[15,15,15,3,3], transform=[[0, -50, 5], [np.pi/4, 0, 0]])
    # World.addObstacle(dims=[15,15,15,3,3], transform=[[0, 0, 55], [np.pi/4, 0, 0]])


    #ADD goal:
    # World.addGoal(dims=[1,1,1,1,1], transform=[[14, -2, 15], [0, 0, 0]])

    # print('Check Goal: ', World.checkGoal())

    # start = time.time()
    # World.plotEnv()
    # end = time.time()
    # print("Plotting: ", end-start)


    # #COMPARE SQ VS GJK!!!!!!!!!

    # start = time.time()
    # World.checkAllCollisionsGJK()
    # end = time.time()
    # print("GJK Check all collisions: ", end-start)

    # start = time.time()
    # World.checkAllCollisionsSQ()
    # end = time.time()
    # print("SQ Check all collisions: ", end-start)


    # start = time.time()
    # World.checkAllCollisionsReverseSQ()
    # end = time.time()
    # print("Reverse SQ Check all collisions: ", end-start)



    #TEST RRT

    # actuatorConfigs = [[0, 0, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0]]
    # goal = [[0, 0, 0], [0, np.pi/6, 0], [0, np.pi/3, 0], [0, -np.pi/2, 0]]
    # startConf = [[0, 0, 0], [0, 1.5, 0], [0, 2, 0], [0, 1, 0]]

    # actuatorConfigs = [[0, 0, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0]]
    # goal = [[0, 0, 0], [0, np.pi/6, 0], [0, np.pi/3, 0], [0, -np.pi/2, 0], [0, np.pi/5, 0]]
    # startConf = [[0, 0, 0], [0, 1.5, 0], [0, 2, 0], [0, 1, 0], [0, 0, 0]]

    # actuatorConfigs = [[0, 0, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0]]
    # goal = [[0, 0, 0], [0, np.pi/6, 0], [0, np.pi/3, 0], [0, -np.pi/2, 0], [0, np.pi/5, 0], [0, -np.pi/5, 0]]
    # startConf = [[0, 0, 0], [0, 1.5, 0], [0, 2, 0], [0, 1, 0], [0, 0, 0], [0, 0, 0]]

    actuatorConfigs = [[0, 0, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0]]
    goalConf = [[0, 0, 0], [0, np.pi/6, 0], [0, np.pi/3, 0], [0, -np.pi/2, 0], [0, np.pi/5, 0], [0, -np.pi/5, 0], [0, -np.pi/8, 0]]
    startConf = [[0, 0, 0], [0, 1.5, 0], [0, 2, 0], [0, 1, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]


    #ACTUATE arm:
    start = time.time()
    World.actuateArm(order=startConf)
    end = time.time()
    print("Actuating Arm: ", end-start)
    
    start = time.time()
    World.plotEnv()
    end = time.time()
    print("Plotting: ", end-start)


    #ACTUATE arm:
    start = time.time()
    World.actuateArm(order=goalConf)
    print(World.Arms[0].links[-1].W[0][-1][-1], World.Arms[0].links[-1].W[1][-1][-1], World.Arms[0].links[-1].W[2][-1][-1])

    end = time.time()
    print("Actuating Arm: ", end-start)

    start = time.time()
    World.plotEnv()
    end = time.time()
    print("Plotting: ", end-start)
#
    print("////////////////////////BEGIN////////////////////////")


    for i in range(8):
        start = time.time()
        MPlanningRRT = RRT(World, actuatorConfigs, goalConf, startConf, 50)
        MPlanningRRT.loop()
        end = time.time()
        print("total count:", MPlanningRRT.count)
        print("RRT: ", end-start)

        MPlanningRRT.getAnglesOut()
        print(MPlanningRRT.OK_path_Orders)

        print("---------------NEXT---------------")

    # #We can access the 
    # #Let's plot the arm nodes
    # mlab.figure(size=(800,800), fgcolor=(0, 0, 0), bgcolor=(1, 1, 1))
 

    # for AO in MPlanningRRT.OK_path_Orders:
    #     print(AO)
    #     World.actuateArm(order=AO)
    #     if AO == MPlanningRRT.OK_path_Orders[0]:
    #         World.Arms[0].plotArm(color_boolean=True, colorVec=(0,1,0))
    #     else:
    #         World.Arms[0].plotArm(color_boolean=True, colorVec=(1,1,0))

    # World.actuateArm(order=startConf)
    # World.Arms[0].plotArm(color_boolean=True, colorVec=(1,0,0))

    # World.actuateArm(order=goalConf)
    # World.Arms[0].plotArm(color_boolean=True, colorVec=(0,0,1))

    # for obstacle in World.Obstacles:
    #     obstacle.plotSQ()
    
    # mlab.show()

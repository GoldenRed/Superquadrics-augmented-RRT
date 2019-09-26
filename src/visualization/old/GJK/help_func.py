import numpy as np
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
        self.W = self.Worig
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

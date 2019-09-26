import numpy as np
from mayavi import mlab

#Author: Yared Efrem Afework <yared94@gmail.com>
#Copyright (C) Yared Efrem Afework, Tritech Technology AB
#Requires Numpy and mayavi
#This files contains the SuperQuadric Class

class SuperQuadric:
    def __init__(self, a1=1, a2=1, a3=1, epsilon1=1, epsilon2=1):
        #Default values give sphere superellipsoid
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3
        self.epsilon1 = epsilon1
        self.epsilon2 = epsilon2

        self.loaded = False #Load basic superllipsoid
        self.transformed = False #transform

    def fexp(self, base, power):
        """the signed exponential function"""
        return (np.sign(base) * (np.abs(base)**power))

    def loadBasicSuperEllipsoid(self, etamin=-np.pi/2, etamax=np.pi/2, omegamin=-np.pi, omegamax=np.pi, n = 80):
        """Method that creates the parametric x,y,z values
        NOTE: SQs are surfaces in 3d space. They are created by taking the "spherical product" of vectors.
        They are reliant on not only one but two orthogonal angle. For a sphere, the first angle (eta) will create a
        semicircle in the plane (-pi/2, pi/2), while the second angle (omega) will create a semicircle in a plane 
        perpendicular to the first semicircle, according to (-pi, pi). This motivates the use of meshgrid.


        """
        self.eta, self.omega = np.mgrid[etamin:etamax:complex(0,n), -np.pi:np.pi:complex(0,n)]
        self.paramx = self.a1 * (self.fexp(np.cos(self.eta), self.epsilon1)) * (self.fexp(np.cos(self.omega), self.epsilon2))
        self.paramy = self.a2 * (self.fexp(np.cos(self.eta), self.epsilon1)) * (self.fexp(np.sin(self.omega), self.epsilon2))
        self.paramz = self.a3 * (self.fexp(np.sin(self.eta), self.epsilon1))
        self.one = np.ones((n,n)) #necessary for homogenous transformation
        self.W = np.array([self.paramx, self.paramy, self.paramz, self.one]) 
        #local coordinate frame -> world frame directly.
        #note that self.W is a 4xnxn tensor.

        self.loaded = True

    def plotSQ(self):
        """Method that plots SQ  in Mayavi"""
        if self.loaded:
            mlab.figure(size=(800,800), fgcolor=(0, 0, 0), bgcolor=(1, 1, 1))
            #mlab.mesh(self.paramx,self.paramy,self.paramz)
            mlab.mesh(self.W[0], self.W[1], self.W[2])
            mlab.outline()
            mlab.title('Superquadric: ' + 'a1: ' + str(self.a1) + '; ' + 'a2: ' \
             + str(self.a2) + '; ' + 'a3: ' + str(self.a3) + '; ' + 'epsilon1: ' \
             + str(self.epsilon1) + '; ' + 'epsilon2: ' + str(self.epsilon2) + '.')
            mlab.axes() #in mayavi/modules/axes.py change self.configure_input_data(self.axes, src.outputs[0]) to self.configure_input_data(self.axes, src.outputs[0].output)
            mlab.show()
        else:
            self.loadBasicSuperEllipsoid()
            self.plotSQ()

    def checkPointCollision(self,pointx,pointy,pointz):
        """Method that checks if a point is in collission"""
        return ((pointx/self.a1)**(2/self.epsilon2) + (pointy/self.a2)**(2/self.epsilon2))**(self.epsilon2/self.epsilon1) + (pointz/self.a3)**(2/self.epsilon1)

    def produceTransformMatOrig(self, translation = [0, 0, 0], eulerAng = [0, 0, 0]):
        """a rotation and translation expressed in ZYZ euler angles
        tx = translation[0]
        ty = translation[1]
        tz = translation[2]
        phi = eulerAng[0]
        theta = eulerAng[1]
        psi = eulerAng[2]"""
        m11 =  np.cos(eulerAng[0])*np.cos(eulerAng[1])*np.cos(eulerAng[2])  - np.sin(eulerAng[0])*np.sin(eulerAng[2])
        m12 = -np.cos(eulerAng[0])*np.cos(eulerAng[1])*np.sin(eulerAng[2])  - np.sin(eulerAng[0])*np.cos(eulerAng[2])
        m13 =  np.cos(eulerAng[0])*np.sin(eulerAng[1])
        m14 =  translation[0]

        m21 =  np.sin(eulerAng[0])*np.cos(eulerAng[1])*np.cos(eulerAng[2])  + np.cos(eulerAng[0])*np.sin(eulerAng[1])
        m22 = -np.sin(eulerAng[0])*np.cos(eulerAng[1])*np.sin(eulerAng[2])  + np.cos(eulerAng[0])*np.cos(eulerAng[1])
        m23 =  np.sin(eulerAng[0])*np.sin(eulerAng[1])
        m24 =  translation[1]

        m31 = -np.sin(eulerAng[1])*np.cos(eulerAng[2])
        m32 =  np.sin(eulerAng[1])*np.sin(eulerAng[2])
        m33 =  np.cos(eulerAng[1])
        m34 =  translation[2]


        self.transMat = np.array([[m11, m12, m13, m14], [m21, m22, m23, m24], [m31, m32, m33, m34], [0, 0, 0, 1]])
        return self.transMat

    def produceTransformMat(self, translation = [0, 0, 0], eulerAng = [0, 0, 0]):
        """a rotation and translation expressed in ZYZ euler angles
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


        self.transMat = np.array([[m11, m12, m13, m14], [m21, m22, m23, m24], [m31, m32, m33, m34], [0, 0, 0, 1]])
        return self.transMat

    def transform(self):
        L,J,K = self.W.shape
        Wnew = np.tensordot(self.transMat, self.W, axes=((1),(0))) #this is equivalent to the commented nested-for loop below:
        # Wnew1 = np.zeros((L,J,K))
        # for j in range(J):
        #     for k in range(K):
        #         for l in range(L):
        #             Wnew1[l,j,k] = self.transMat[l,0]*self.W[0,j,k] + self.transMat[l,1]*self.W[1,j,k] + self.transMat[l,2]*self.W[2,j,k] + self.transMat[l,3]*self.W[3,j,k]
        self.W = Wnew
        print("Transformed.")





    def checkPointCollisionPostTransformation(self,pointx,pointy,pointz):
        """Method that checks if a point is in collission"""
        nx = self.transMat[1][0]
        ny = self.transMat[2][0]
        nz = self.transMat[3][0]

        ox = self.transMat[1][1]
        oy = self.transMat[2][1]
        oz = self.transMat[3][1]

        ax = self.transMat[1][2]
        ay = self.transMat[2][2]
        az = self.transMat[3][2]

        px = self.transMat[1][3] 
        py = self.transMat[2][3]
        pz = self.transMat[3][3]

        expression1 = nx*pointx + ny*pointy + nz*pointz - px*nx - py*ny - pz*nz
        expression2 = ox*pointx + oy*pointy + oz*pointz - px*ox - py*oy - pz*oz
        expression3 = ax*pointx + ay*pointy + az*pointz - px*ax - py*ay - pz*az
        return ((expression1/self.a1)**(2/self.epsilon2) + (expression2/self.a2)**(2/self.epsilon2))**(self.epsilon2/self.epsilon1) + (expression3/self.a3)**(2/self.epsilon1)
 

if __name__ == '__main__':
    sq = SuperQuadric()
    sq.plotSQ()
    print(sq.produceTransformMat(translation=[0,0,0], eulerAng=[np.pi/4, 0, 0]))
    sq.transform()
    sq.plotSQ()
    print(sq.produceTransformMat(translation=[0,0,0], eulerAng=[0, np.pi/4, 0]))
    sq.transform()
    sq.plotSQ()
    print(sq.produceTransformMat(translation=[0,0,0], eulerAng=[0, 0, np.pi/4]))
    sq.transform()
    sq.plotSQ()

    print(sq.checkPointCollision(1,0,0))





















# import numpy as np
# from mayavi import mlab

# #Author: Yared Efrem Afework <yared94@gmail.com>
# #Copyright (C) Yared Efrem Afework, Tritech Technology AB
# #Requires Numpy and mayavi

# #v1: allowed for basic SQ rendering and check
# #v2: allowed for transformation (trans+rot) of basic SQ and collision check
# #v3: allow for basic initialisation of an arm and collision check

# #Current Todo: Figure out how to actuate the arm!

# class SuperQuadric:
#     def __init__(self, a1=1, a2=1, a3=1, epsilon1=1, epsilon2=1):
#         #Default values give sphere superellipsoid
#         self.a1 = a1
#         self.a2 = a2
#         self.a3 = a3
#         self.epsilon1 = epsilon1
#         self.epsilon2 = epsilon2

#         self.loadBasicSuperEllipsoid()
#         self.transMat = self.produceTransformMat([0,0,0], [0,0,0]) #this variable holds the overall transformation

#     def fexp(self, base, power):
#         """the signed exponential function"""
#         return (np.sign(base) * (np.abs(base)**power))

#     def loadBasicSuperEllipsoid(self, etamin=-np.pi/2, etamax=np.pi/2, omegamin=-np.pi, omegamax=np.pi, n = 5):
#         """Method that creates the parametric x,y,z values
#         NOTE: SQs are surfaces in 3d space. They are created by taking the "spherical product" of vectors.
#         They are reliant on not only one but two orthogonal angle. For a sphere, the first angle (eta) will create a
#         semicircle in the plane (-pi/2, pi/2), while the second angle (omega) will create a semicircle in a plane 
#         perpendicular to the first semicircle, according to (-pi, pi). This motivates the use of meshgrid.


#         """
#         self.eta, self.omega = np.mgrid[etamin:etamax:complex(0,n), -np.pi:np.pi:complex(0,n)]
#         self.paramx = self.a1 * (self.fexp(np.cos(self.eta), self.epsilon1)) * (self.fexp(np.cos(self.omega), self.epsilon2))
#         self.paramy = self.a2 * (self.fexp(np.cos(self.eta), self.epsilon1)) * (self.fexp(np.sin(self.omega), self.epsilon2))
#         self.paramz = self.a3 * (self.fexp(np.sin(self.eta), self.epsilon1))
#         self.one = np.ones((n,n)) #necessary for homogenous transformation
#         self.Worig = np.array([self.paramx, self.paramy, self.paramz, self.one]) 
#         #local coordinate frame -> world frame directly.
#         #note that self.W is a 4xnxn tensor.

#     def plotSQ(self):
#         """Method that plots SQ  in Mayavi"""
#         if self.loaded:
#             mlab.figure(size=(800,800), fgcolor=(0, 0, 0), bgcolor=(1, 1, 1))
#             #mlab.mesh(self.paramx,self.paramy,self.paramz)
#             mlab.mesh(self.W[0], self.W[1], self.W[2])
#             mlab.outline()
#             mlab.title('Superquadric: ' + 'a1: ' + str(self.a1) + '; ' + 'a2: ' \
#              + str(self.a2) + '; ' + 'a3: ' + str(self.a3) + '; ' + 'epsilon1: ' \
#              + str(self.epsilon1) + '; ' + 'epsilon2: ' + str(self.epsilon2) + '.')
#             mlab.axes() #in mayavi/modules/axes.py change self.configure_input_data(self.axes, src.outputs[0]) to self.configure_input_data(self.axes, src.outputs[0].output)
#             mlab.show()
#         else:
#             self.loadBasicSuperEllipsoid()
#             self.plotSQ()

#     def checkPointCollision(self,pointx,pointy,pointz):
#         """Method that checks if a point is in collission"""
#         return ((pointx/self.a1)**(2/self.epsilon2) + (pointy/self.a2)**(2/self.epsilon2))**(self.epsilon2/self.epsilon1) + (pointz/self.a3)**(2/self.epsilon1)

#     def produceTransformMat(self, translation, eulerAng):
#         """a rotation and translation expressed in ZYZ euler angles
#         tx = translation[0]
#         ty = translation[1]
#         tz = translation[2]
#         phi = eulerAng[0]
#         theta = eulerAng[1]
#         psi = eulerAng[2]"""
#         m11 =  np.cos(eulerAng[0])*np.cos(eulerAng[1])*np.cos(eulerAng[2])  - np.sin(eulerAng[0])*np.sin(eulerAng[2])
#         m12 = -np.cos(eulerAng[0])*np.cos(eulerAng[1])*np.sin(eulerAng[2])  - np.sin(eulerAng[0])*np.cos(eulerAng[2])
#         m13 =  np.cos(eulerAng[0])*np.sin(eulerAng[1])
#         m14 =  translation[0]

#         m21 =  np.sin(eulerAng[0])*np.cos(eulerAng[1])*np.cos(eulerAng[2])  + np.cos(eulerAng[0])*np.sin(eulerAng[1])
#         m22 = -np.sin(eulerAng[0])*np.cos(eulerAng[1])*np.sin(eulerAng[2])  + np.cos(eulerAng[0])*np.cos(eulerAng[1])
#         m23 =  np.sin(eulerAng[0])*np.sin(eulerAng[1])
#         m24 =  translation[1]

#         m31 = -np.sin(eulerAng[1])*np.cos(eulerAng[2])
#         m32 =  np.sin(eulerAng[1])*np.sin(eulerAng[2])
#         m33 =  np.cos(eulerAng[1])
#         m34 =  translation[2]


#         transMat = np.array([[m11, m12, m13, m14], [m21, m22, m23, m24], [m31, m32, m33, m34], [0, 0, 0, 1]])
#         return transMat

#     def transform(self, translation = [0, 0, 0], rotation = [0, 0, 0], incomingOutsideTransMat=False, outsideTransMat=None):
#         """Transforms all of the x,y,z points  homogenously. Keeps track of previous transformations through self.transMat
#         outsideTransMat is used to keep the amount of transformations low. It is used for getting the link to where the previous 
#         links are located, but then after that the link can be shifted by itself"""
#         if incomingOutsideTransMat == False:
#             print("...........")
#             tempTransMat = self.produceTransformMat(translation, rotation)
#             print("tTM: ",tempTransMat)
#             print("PRE self.tM: ", self.transMat)       
#             self.transMat = np.matmul(tempTransMat,self.transMat) #Maybe should be multiplied in the reverse direction
#             print("POST self.tM: ", self.transMat)  
#             print(",,,,,,,,,,,")     
#         else:
#             self.transMat = np.matmul(outsideTransMat,self.transMat)

#         Wnew = np.tensordot(self.transMat, self.Worig, axes=((1),(0))) #this is equivalent to the commented nested-for loop below:

#         #L,J,K = self.Worig.shape
#         # Wnew1 = np.zeros((L,J,K))
#         # for j in range(J):
#         #     for k in range(K):
#         #         for l in range(L):
#         #             Wnew1[l,j,k] = self.transMat[l,0]*self.W[0,j,k] + self.transMat[l,1]*self.W[1,j,k] + self.transMat[l,2]*self.W[2,j,k] + self.transMat[l,3]*self.W[3,j,k]
#         self.W = Wnew
#         #print("Transformed.")

#     def reset(self):
#         self.transMat = self.produceTransformMat([0,0,0], [0,0,0])


#     def checkPointCollisionPostTransformation(self,pointx,pointy,pointz):
#         """Method that checks if a point is in collission"""
#         nx = self.transMat[1][0]
#         ny = self.transMat[2][0]
#         nz = self.transMat[3][0]

#         ox = self.transMat[1][1]
#         oy = self.transMat[2][1]
#         oz = self.transMat[3][1]

#         ax = self.transMat[1][2]
#         ay = self.transMat[2][2]
#         az = self.transMat[3][2]

#         px = self.transMat[1][3] 
#         py = self.transMat[2][3]
#         pz = self.transMat[3][3]

#         expression1 = nx*pointx + ny*pointy + nz*pointz - px*nx - py*ny - pz*nz
#         expression2 = ox*pointx + oy*pointy + oz*pointz - px*ox - py*oy - pz*oz
#         expression3 = ax*pointx + ay*pointy + az*pointz - px*ax - py*ay - pz*az
#         return ((expression1/self.a1)**(2/self.epsilon2) + (expression2/self.a2)**(2/self.epsilon2))**(self.epsilon2/self.epsilon1) + (expression3/self.a3)**(2/self.epsilon1)


# class Arm:
#     def __init__(self, base=SuperQuadric(1,1,1,1,1)):
#         self.links = []
#         self.configuration = []
#         self.addLink(base)

#     def actuateArm(self):
#         """Once the arm has been initialised we only want to be able to rotate the joints"""

#         #reset the links from their previous configuration (e.g., )
#         # for link in self.links:
#         #     link.reset()

#         # #let's try rotating every link by pi/4 radians

#         # for i in range(len(self.links)):
#         #     linkVec = self.links[i:-1]
#         #     print("i: ", i, " len(linkVec): ", len(linkVec))
#         #     self.cascadedRotation([np.pi/3, 0, 0], linkVec)

# #        for i in range(1, len(self.links)):

#         for i in range(len(self.links)):
#             link_prev = self.links[i-1]
#             link_next = self.links[i]

#             print("PREV TRANSMAT:", link_prev.transMat)
            
#             link_next.transform(incomingOutsideTransMat=True, outsideTransMat=link_prev.transMat)
#             link_next.transform(incomingOutsideTransMat=False, rotation=[0, np.pi/4, 0])


#     # def cascadedRotation(self, rotVec, linkVec):
#     #     """to rotate an an n-link arm, n(n+1)/2 individual rotations need to be done (e.g. a 6 link arm first needs to do 6 rotations, then 5 on the upper links, then 4, then ..., etc"""
#     #     for primitive in linkVec:
#     #         primitive.transform(incomingOutsideTransMat=False, rotation=rotVec)



#     def addLink(self, primitive, translation=[0,0,0], rotation=[0,0,0]):
#         """Don't use this...
#         If you want to directly add the SQ primitive, translation and rotation.
#         Is called by default for the original base piece"""
#         primitive.loadBasicSuperEllipsoid()
#         primitive.transform(translation,rotation)
#         self.configuration.append([translation, rotation])
#         self.links.append(primitive)

#     def newStandardLink(self, zdimlength=10):
#         """Will add "standard" link of a certain height"""
#         primitive = SuperQuadric(1,1,zdimlength, 0.2, 0.2) #Standard link
#         #then shift it to the right height:
#         self.provideInitialTranslation(primitive,self.links[-1])
#         #add it to the list of links
#         self.links.append(primitive)

#     def provideInitialTranslation(self, primitive, prevlink):
#         """The idea is that the arm begins with all links pointing STRAIGHT up in the z-axis"""
        
#         #get the current link to the previous links position:
#         primitive.transform(incomingOutsideTransMat=True, outsideTransMat=prevlink.transMat)

#         #current link needs to shifted by previous link's a3 value and its own a3 value to stop being inside it
#         shift = primitive.a3 + prevlink.a3
#         primitive.transform(translation=[0,0,shift], rotation=[0,0,0], incomingOutsideTransMat=False)


#     def reConfigureArm(self):
#         return 0

#     def checkCollision(self, pointx, pointy, pointz):
#         F_list = []
#         for link in self.links:
#             F_list.append(link.checkPointCollisionPostTransformation(pointx, pointy, pointz))
#         return F_list

#     def plotArm(self):
#         """Method that plots SQ  in Mayavi"""
#         #If you get error with the axes, do the following replacement:
#         #in mayavi/modules/axes.py change self.configure_input_data(self.axes, src.outputs[0]) to self.configure_input_data(self.axes, src.outputs[0].output)
#         mlab.figure(size=(800,800), fgcolor=(0, 0, 0), bgcolor=(1, 1, 1))
#         #mlab.mesh(self.paramx,self.paramy,self.paramz)

#         for primitive in self.links:
#             mlab.mesh(primitive.W[0], primitive.W[1], primitive.W[2])
#             print(primitive.W[3])
#             if primitive == self.links[0]:
#                 #give the base SQ axes and outline
#                 mlab.axes()
#                 mlab.outline()
#         mlab.title('Arm')
#         mlab.show()
#         return 0

# if __name__ == '__main__':
#     import time
#     start = time.time()
#     RobotArm = Arm()
#     end = time.time()
#     print("Creating arm: ", end-start)


#     # for i in range(1):
#     #     start = time.time()
#     #     RobotArm.newStandardLink()
#     #     end = time.time()
#     #     print("Adding Link: ", end-start)




#     start = time.time()
#     Fs = RobotArm.checkCollision(1,0,0)
#     end = time.time()
#     print("Collision Check: ", end-start)
#     print(Fs)


#     start = time.time()
#     RobotArm.plotArm()
#     end = time.time()
#     print("Plotting: ", end-start)

#     start = time.time()
#     RobotArm.actuateArm()
#     end = time.time()
#     print("Actuating Arm: ", end-start)

#     print("\n \n \n \n \n \n")

#     start = time.time()
#     RobotArm.plotArm()
#     end = time.time()
#     print("Plotting: ", end-start)

#     print("\n \n \n \n \n \n")

#     A = SuperQuadric()
#     print(A.produceTransformMat(translation=[0, 0, 0], eulerAng=[0, np.pi/4,0]))




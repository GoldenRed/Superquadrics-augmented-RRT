import numpy as np
from mayavi import mlab

#Author: Yared Efrem Afework <yared94@gmail.com>
#Copyright (C) Yared Efrem Afework, Tritech Technology AB
#This files contains the SuperQuadric Class

class SuperQuadric:
    def __init__(self, a1=1, a2=1, a3=1, epsilon1=1, epsilon2=1):
        #Default values give sphere superellipsoid
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3
        self.epsilon1 = epsilon1
        self.epsilon2 = epsilon2
        self.loaded = False

    def fexp(self, base, power):
        """the signed exponential function"""
        return (np.sign(base) * (np.abs(base)**power))

    def loadBasicSuperEllipsoid(self, etamin=-np.pi/2, etamax=np.pi/2, omegamin=-np.pi, omegamax=np.pi, n = 80):
        """Method that creates the parametric x,y,z values"""
        self.eta, self.omega = np.mgrid[etamin:etamax:complex(0,n), -np.pi:np.pi:complex(0,n)]
        self.paramx = self.a1 * (self.fexp(np.cos(self.eta), self.epsilon1)) * (self.fexp(np.cos(self.omega), self.epsilon2))
        self.paramy = self.a2 * (self.fexp(np.cos(self.eta), self.epsilon1)) * (self.fexp(np.sin(self.omega), self.epsilon2))
        self.paramz = self.a3 * (self.fexp(np.sin(self.eta), self.epsilon1))
        self.loaded = True

    def plotSQ(self):
        """Method that plots SQ  in Mayavi"""
        if self.loaded:
            mlab.figure(fgcolor=(0, 0, 0), bgcolor=(1, 1, 1))
            mlab.mesh(self.paramx,self.paramy,self.paramz)
            mlab.show()
        else:
            self.loadBasicSuperEllipsoid()
            self.plotSQ()

    def checkPointCollision(self,pointx,pointy,pointz):
        """Method that checks if a point is in collission"""
        return ((pointx/self.a1)**(2/self.epsilon2) + (pointy/self.a2)**(2/self.epsilon2))**(self.epsilon2/self.epsilon1) + (pointz/self.a3)**(2/self.epsilon1)

if __name__ == '__main__':
    sq = SuperQuadric()
    sq.plotSQ()
    print(sq.checkPointCollision(1,0,0))


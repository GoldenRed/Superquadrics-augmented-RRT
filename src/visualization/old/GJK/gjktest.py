import numpy as np
from mayavi import mlab
from help_func import SuperQuadric



# mlab.figure(size=(800,800), fgcolor=(0, 0, 0), bgcolor=(1, 1, 1))
# mlab.points3d(A.W[0], A.W[1], A.W[2], color=(1,1,1))
# mlab.points3d(B.W[0], B.W[1], B.W[2], color=(1,1,0))
# mlab.show()







def GJKloop(link, obs):
    """Based on  Casey Muratori's video"""
    iterations = 0
    eTol = 0.1
    S = SupportMapFunc(link,obs) #V random in here, returns point in random direction
    W = [] #W contains all of our verteces
    W.append(S)
    V = -1*S #returns SO = O - S
    #print('S:', S, 'V:', V)

    while(1):
        iterations = iterations + 1
        if(iterations > 50):
            print('So many iterations!', iterations, " It's probably true...")
            return True
        w = SupportMapFunc(link, obs, V)
        if( (abs(w[0]) < eTol) and (abs(w[1]) < eTol) and (abs(w[2]) < eTol)):
            print('Look at w:',w,'! It is probably the Origin')
            return True
        
        print('New point w:',w)
        print('Current V:',V)
        if np.dot(w, V) <= 0:
            print('No Intersection')
            return False
        else:
            W.append(w)
            print('Current W:', len(W), W)
            W, V = DoSimplex(W,V)
            if len(V) == 0:
                return True

def DoSimplex(W, V):
    if len(W) == 0:
        print('Impossible n=0 case')
        return W, V

    elif len(W) == 1:
        print('Impossible n=1 case')
        return W, V

    elif len(W) == 2:
        print('n=2 case')
        return TwoPoints(W,V)

    elif len(W) == 3:
        print('n=3 case')
        return ThreePoints(W,V)

    elif len(W) == 4:
        print('n=4 case')
        return FourPoints(W,V)

    else:
        print('n>4 case')
        return

def TwoPoints(W,V):
    #Line Scenario
    #B = W[0] prev point
    #A = W[1] new point
    A = W[-1] #last added point
    AO = -1*A #direction to origin from A
    B = W[-2] #previous point
    AB = B-A #line from new point to old
    #There is no BO because we have previously ruled out that region (the region closest to B)
    if np.dot(AB, AO)>0:
        print('TwoPoints, O is in AB-region')
        newV = np.cross(np.cross(AB,AO),AB)
        return W, newV
    else:
        print('TwoPoints, O is in A-region')
        newW = [A]
        newV = AO
        return newW, newV

#QUESTION: If we return newW, going by Casey's notation should I return it as [A, B, C] or [C, B, A]??
    
def ThreePoints(W,V):
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
            print('ThreePoints, O is in AC-region')
            newW = [C, A]
            newV = np.cross( np.cross(AC,AO), AC)
            return newW, newV
        else:
            #Is it in AB-edge or A region?
            return STARFUNCTION(A, B, AO, AB)

    else: #It is not perpendicular to AC-edge region
        if np.dot(np.cross(AB,ABC),AO) > 0: #We check if it is perpendicular to AB-region
            return STARFUNCTION(A, B, AO, AB)
        else:
            #It must be above or below the triangle (we have ruled out BC, B, C regions already)
            if np.dot(ABC, AO) > 0:
                #The origin is below the triangle
                #Case 2
                #newW = [A, B, C]
                print('ThreePoints, O is below ABC')
                #newW = [C, B, A]
                #flip the triangle:
                newW = [B, C, A]
                newV = ABC
                return newW, newV
            else:
                #The origin is above the triangle:
                #Case 3
                #newW = [A, C, B] #we flip the triangle
                print('ThreePoints, O is above ABC')
                newW = [C, B, A]
                newV = -1*ABC
                return newW, newV


def STARFUNCTION(A, B, AO, AB):
    """because Casey Muratori used a star in his video for this """
    if np.dot(AB, AO) > 0: #is the point in AB-region?
        #The point is in AB region
        #Case 4
        #newW = [A,B]
        print('ThreePoints, O is in AB-region')
        newW = [B,A]
        newV = np.cross(np.cross(AB,AO), AB)
        return newW, newV
    else: 
        #The point is in the A region
        #Case 5
        print('ThreePoints, O is in A-region')
        newW = [A]
        newV = AO
        return newW, newV





def FourPoints(W,V):
    #TETRAHEDRAL SCENARIO
    #But we're basically looking at the 3 (out of 4) faces of the tetrahedron separately (since we already check that). The code for checking the A-point region as well as some of the 

    A = W[-1]
    B = W[-2]
    C = W[-3]   
    D = W[-4]

    #We don't check the DCB triangle

    CBA_BOOL = ThreePoints_TetraCase([C, B, A], V)
    BDA_BOOL = ThreePoints_TetraCase([B, D, A], V)
    DCA_BOOL = ThreePoints_TetraCase([D, C, A], V)

    #111
    if (CBA_BOOL and BDA_BOOL and DCA_BOOL) == 1: #111
        print('Origin ENCAPSULATED!!!!!!!!!!!!!!')
        return W, []


#These three tell us to kick out one point:
    
    elif(CBA_BOOL and (not BDA_BOOL) and (not DCA_BOOL)) == 1:#100
        print('Kick out D')
        newW = [C, B, A]
        #search in the opposite direction of D?
        newV = -1*D
        return newW, newV

    elif((not CBA_BOOL) and (not BDA_BOOL) and DCA_BOOL) == 1: #001
        print('Kick out B')
        newW = [D, C, A]
        #search in the opposite direction of B?
        newV = -1*B
        return newW, newV

    elif((not CBA_BOOL) and BDA_BOOL and (not DCA_BOOL)) == 1: #010
        print('Kick out C')
        newW = [D, B, A]
        #search in the opposite direction of C?
        newV = -1*C
        return newW, newV


#This one tells us to kick out B,C,D and start over 
    elif((not CBA_BOOL) and (not BDA_BOOL) and (not DCA_BOOL)) == 1: #000
        print('Kick out B, C, D!')
        print('Check Out: NOT_DCB_BOOL:', (not ThreePoints_TetraCase([D, C, B], V)))
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
        #in this case it is probably trapped in some weird edge case? I dunno
        print('WTF?')
        print('Check Out: NOT_DCB_BOOL:', (not ThreePoints_TetraCase([D, C, B], V)))





def ThreePoints_TetraCase(W,V):
    #Basically we are ONLY interested in knowing if the origin is in the inside-direction or outside-direction of the triangle
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


# def STARFUNCTION_TetraCase(A, B, AO, AB)
# """because Casey Muratori used a star in his video for this """
# #Checks the region of points closest to A
# if np.dot(AB, AO) > 0:
#   #Case 4
#   newW = [A,B]
#   newV = np.cross(AB,np.cross(AO, AB))
#   return newW, newV
# else:
#   #Case 5
#   newW = [A]
#   newV = AO
#   return newW, newV






def SupportMapFunc(link, obs, V = np.random.rand(3,1)):

    S_Link = FurthestPointInDirection(link, V)
    S_Obs = FurthestPointInDirection(obs, V, reverse=True)

    return S_Link - S_Obs

def FurthestPointInDirection(convexSet, V, reverse=False):
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







if __name__ == '__main__':
    A = SuperQuadric(1,1,1,1,1,n=20)
    B = SuperQuadric(1,1,1,1,1,n=20)
    B.transform(translation=[10,10,10])
    vv = np.array([1,1,0])
    print(SupportMapFunc(A, B, vv))
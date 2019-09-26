# SuperQuadrics-augmented RRT

This thesis was about examining the possibility of using superquadrics (a general family of surfaces in 3D space) to do collision detection in the RRT motion planning algorithm.


## Installation
The code requires:
* [Numpy](https://www.numpy.org/) for the mathematical calculations
* [Mayavi](https://docs.enthought.com/mayavi/mayavi/installation.html) for the visualization.



## Classes
WorldSim.py contains the following classes:
* Environment() class. Its purpose is to abstract away the world that is being simulated.
* Arm() class. Is used to organize individual SQ arm links and base into an arm, with relevant methods for ensuring their proper relative distance position is maintained. 
* SuperQuadric() class (Superellipsoid). The basic class for all the objects, as all objects are represnted using them. Has built-in point-wise collision detection methods.
* GJK_Collision_Module() class. Takes in two SQ objects (i.e., two point clouds) and checks intersections between them using the Gilbert-Johnson-Keerthi distance algorithm.
* RRT() class. It makes use of an instance of Environment() and performs the RRT algorithm on that world.
** (SuperToroid() class. Also a form of SQ, this class is available for plotting SuperToroid SQs. It is not interchangeable with the SuperQuadric class)


## Usage

Import the file WorldSim.py (under src) into your program. All of Numpy is imported as np and mlab is imported from Mayavi in the WorldSim.py.

```python
from WorldSim import *
```

Initiate an Environment object:

```python
World = Environment()
```
This object needs to have an arm inserted:
```python
World.addArm()
```
This arm needs to have links added:
```python
World.Arms[0].newStandardLink()  #Default values: xdimlength=1, ydimlength=1, zdimlength=10, eps1shape=0.2, eps2shape=0.2
```
The link is a superquadric (SQ) object with the aforementioned default dimensions and shape parameters. The arm object will know to concatenate the link to the arm. Repeat until enough 

If you want an obstacle added:
```python
World.addObstacle() #dims=[1,1,1,1,1], transform=[[0,0,0], [0,0,0]]
```
The new obstacle is also an SQ object, with default alues provided. dims=[] contains the five dimension and shapre parameters, whereas transform=[[],[]] contains the translational (px, py, pz) and rotational (phi, theta, psi) parameters allowing for the obstacle to be placed wherever is needed in the world.

To actuate the arm:
```python
armOrder = ~[~[0, [[0, 3.14/2, 0]] #rotates the first link's theta angle 180 degrees
World.actuateArm(order=armOrder) #number=0,
```
Use order=[[...]] in the actuateArm method of Environment() to actuate the arm in accordance with the armOrder. The method has as default the first arm entered (i.e. the 0th arm, World.Arms[0]), but if there are multiple arms number can be passed in as a different value. Note that armOrder is applicable to one arm and needs to match the link length of the arm such that the order and the degrees of freedom match up.

To check if there any collisions in the world, the following lines can be used:
```python
World.checkAllCollisionsGJK()
World.checkAllCollisionsSQ()
World.checkAllCollisionsReverseSQ()
```
These methods perform the collision detection using either the GJK algorithm or the SQ based algorithms. The difference between the "obverse" and reverse version is that the former explicitly represents the obstacles and checks them using the arm links' implicit functions, while the reverse algorithm does the reverse (explicit links, implicit obstacles).

To plot the world (using the Mayavi librari), simply use the following:
```python
World.plotEnv()
```
It uses the explicit points of the SQ objects. The point density is set globally by changing the SuperQuadric class' default n parameter, which defines the number of increments in the spherical product. In total, there are n^2 number of points in the object.


To run the RRT algorithm, initialize an RRT object:
```python
MPlanningRRT = RRT(World, actuatorConfigs, goalConf, startConf, 50)
```
The RRT class init requires the Environment() object (World), the actuatorConfigs, the goal configuration and the start configuration. The actuatorConfigs defines what angle the RRT algorithm is allowed to vary for each joint. In the thesis, only revolute joints are looked at and the base is considered fixed. Thus, the following matrix was used (for 6 DoF arm): [[0, 0, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0]]]], where the 1 informs that the algorithm can vary that angle in that joint. 

Finally, the last number (50) refers to how often the algorithm will sample the goal configuration instead of sampling randomly. 50 in this case means every 50 iteration.

Use:

```python
MPlanningRRT.loop()
```

To start the loop, and then once it is done:
```python
MPlanningRRT.getAnglesOut()
print(MPlanningRRT.OK_path_Orders)
```

This will have the algorithm print out the path.

If you want to plot the results of the algorithm you can use the following code snippet:

```python
mlab.figure(size=(800,800), fgcolor=(0, 0, 0), bgcolor=(1, 1, 1))
for AO in MPlanningRRT.OK_path_Orders:
    print(AO)
    World.actuateArm(order=AO)
    if AO == MPlanningRRT.OK_path_Orders[0]:
        World.Arms[0].plotArm(color_boolean=True, colorVec=(0,1,0))
    else:
        World.Arms[0].plotArm(color_boolean=True, colorVec=(1,1,0))
World.actuateArm(order=startConf)
World.Arms[0].plotArm(color_boolean=True, colorVec=(1,0,0))
World.actuateArm(order=goalConf)
World.Arms[0].plotArm(color_boolean=True, colorVec=(0,0,1))
for obstacle in World.Obstacles:
    obstacle.plotSQ()
mlab.show()
```

It creates a Mayavi figure, actuates the arm into each position before running the internal method plotArm to populate the figure with that specific pose. It assigns intermediary poses the color yellow, the start configuration red and the goal configuration green. The actual final accepted configuration (a configuration where the end-effector of the robot arm is inside the same region as that of the goal configuration's) is plotted in blue.  The obstacles in the world are also populated to the figure, before finally mlab.show() is run to start the visualization.
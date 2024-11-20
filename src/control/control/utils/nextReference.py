import numpy as np

#The class takes in a State object 'currentState', a Path object 'path', and an int 'index' 
#which indicates the next point on the path that the sub is intended for. Line 13 forces
#the index to be 0 if initially given less than 0, and it will be fixed at the max index when it's
#at the end of path.

#!!! IS THERE A .SIZE() METHOD FOR PATH TYPE???!!!

class nextReference:
    def __init__ (self, cur_state, path):
        self.cur_state = currentState
        self.path = path
        self.index = 0
        
<<<<<<< HEAD
<<<<<<< HEAD
=======
    
>>>>>>> 9f74931 (updates nextState to nextReference)
=======
>>>>>>> 5adde3d (updates nextState to nextReference)
    def initializer (self):
        self.path = None
        self.index = 0

<<<<<<< HEAD
<<<<<<< HEAD
    #naive
    def changePath (self, newPath, newIndex):
        self.path = newPath
        if (newIndex < newPath.size()):
            self.index = newIndex
        else:
            self.index = newPath.size() - 1
=======
    #in progress
    def changePath (self, newPath):
        newIndex = self.index
        if newIndex + 1 < newPath.size():
            if newPath[newIndex]:
        self.path = newPath
>>>>>>> 9f74931 (updates nextState to nextReference)
=======
    #naive
    def changePath (self, newPath, newIndex):
        self.path = newPath
        if (newIndex < newPath.size()):
            self.index = newIndex
        else:
            self.index = newPath.size() - 1
>>>>>>> 5adde3d (updates nextState to nextReference)

    def changeState (self, newState):
        self.cur_state = newState

<<<<<<< HEAD
<<<<<<< HEAD
    #index indicate the current path target
=======
>>>>>>> 9f74931 (updates nextState to nextReference)
=======
    #index indicate the current path target
>>>>>>> 5adde3d (updates nextState to nextReference)
    def getNextReference (self):
        if index < 0:
            index = 0
        
<<<<<<< HEAD
<<<<<<< HEAD
        distanceToNext = numpy.linalg.norm(currentState - self.path[self.index])
        distanceToPrevious = numpy.linalg.norm(currentState - self.path[self.index] - 1)

        if (distanceToNext < distanceToPrevious and self.index < path.size() - 1):
            index += 1

        return path[self.index] 


<<<<<<< HEAD
=======
        currentPoint = cur_state.pos
        
        distanceToNext = sqrt((currentState.x - self.path[self.index].x) ** 2 + 
                        (currentState.y - self.path[self.index].y ** 2) + 
                        (currentState.z - self.path[self.index].z) ** 2)
=======
        distanceToNext = numpy.linalg.norm(currentState - self.path[self.index])
        distanceToPrevious = numpy.linalg.norm(currentState - self.path[self.index] - 1)
>>>>>>> 5adde3d (updates nextState to nextReference)

        if (distanceToNext < distanceToPrevious and self.index < path.size() - 1):
            index += 1

        return path[self.index] 

<<<<<<< HEAD
    def getDistance(point1, point2):
        return sqrt((point1.x - point2.x) ** 2 + 
                        (point1.y - point2.y ** 2) + 
                        (point1.z - point2.z) ** 2)

    
>>>>>>> 9f74931 (updates nextState to nextReference)
=======
    #
>>>>>>> 5adde3d (updates nextState to nextReference)
=======
>>>>>>> 766c5d4 (Path follower)

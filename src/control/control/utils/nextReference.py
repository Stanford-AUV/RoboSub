import numpy as np

<<<<<<< HEAD
#The class takes in a State object 'currentState', a Path object 'path', and an int 'index' 
#which indicates the next point on the path that the sub is intended for. Line 13 forces
#the index to be 0 if initially given less than 0, and it will be fixed at the max index when it's
#at the end of path.

#!!! IS THERE A .SIZE() METHOD FOR PATH TYPE???!!!

class nextReference:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    def __init__ (self, currentState, path):
=======
    def __init__ (self, cur_state, path):
>>>>>>> 9f74931 (updates nextState to nextReference)
=======
    def __init__ (self, currentState, path):
>>>>>>> a0c291a (Update nextReference.py (#48))
=======
    def __init__ (self, cur_state, path):
>>>>>>> 9f74931 (updates nextState to nextReference)
=======
    def __init__ (self, currentState, path):
>>>>>>> a0c291a (Update nextReference.py (#48))
=======
    def __init__ (self, currentState, path):
>>>>>>> a0c291a (Update nextReference.py (#48))
        self.cur_state = currentState
        self.path = path
        self.index = 0
        
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
    
>>>>>>> 9f74931 (updates nextState to nextReference)
=======
>>>>>>> 5adde3d (updates nextState to nextReference)
=======
    
>>>>>>> 9f74931 (updates nextState to nextReference)
=======
>>>>>>> 5adde3d (updates nextState to nextReference)
=======
    
>>>>>>> 9f74931 (updates nextState to nextReference)
=======
>>>>>>> 5adde3d (updates nextState to nextReference)
    def initializer (self):
        self.path = None
        self.index = 0

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
>>>>>>> 9f74931 (updates nextState to nextReference)
=======
>>>>>>> 9f74931 (updates nextState to nextReference)
    #in progress
    def changePath (self, newPath):
        newIndex = self.index
        if newIndex + 1 < newPath.size():
            if newPath[newIndex]:
        self.path = newPath
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> 9f74931 (updates nextState to nextReference)
=======
    #naive
    def changePath (self, newPath, newIndex):
        self.path = newPath
=======
    #naive
    def changePath (self, newPath, newIndex):
        self.path = newPath
>>>>>>> 5adde3d (updates nextState to nextReference)
=======
    #naive
    def changePath (self, newPath, newIndex):
        self.path = newPath
>>>>>>> 5adde3d (updates nextState to nextReference)
        if (newIndex < newPath.size()):
            self.index = newIndex
        else:
            self.index = newPath.size() - 1
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> 5adde3d (updates nextState to nextReference)
=======
>>>>>>> 9f74931 (updates nextState to nextReference)
=======
>>>>>>> 5adde3d (updates nextState to nextReference)
=======
>>>>>>> 9f74931 (updates nextState to nextReference)
=======
>>>>>>> 5adde3d (updates nextState to nextReference)

    def changeState (self, newState):
        self.cur_state = newState

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    #index indicate the current path target
=======
>>>>>>> 9f74931 (updates nextState to nextReference)
=======
    #index indicate the current path target
>>>>>>> 5adde3d (updates nextState to nextReference)
=======
>>>>>>> 9f74931 (updates nextState to nextReference)
=======
    #index indicate the current path target
>>>>>>> 5adde3d (updates nextState to nextReference)
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        distanceToNext = numpy.linalg.norm(currentState - self.path[self.index])
        distanceToPrevious = numpy.linalg.norm(currentState - self.path[self.index] - 1)

        if (distanceToNext < distanceToPrevious and self.index < path.size() - 1):
            index += 1

        return path[self.index] 


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> 9f74931 (updates nextState to nextReference)
=======
>>>>>>> 9f74931 (updates nextState to nextReference)
        currentPoint = cur_state.pos
        
        distanceToNext = sqrt((currentState.x - self.path[self.index].x) ** 2 + 
                        (currentState.y - self.path[self.index].y ** 2) + 
                        (currentState.z - self.path[self.index].z) ** 2)
<<<<<<< HEAD
<<<<<<< HEAD
=======
        distanceToNext = numpy.linalg.norm(currentState - self.path[self.index])
        distanceToPrevious = numpy.linalg.norm(currentState - self.path[self.index] - 1)
>>>>>>> 5adde3d (updates nextState to nextReference)

        if (distanceToNext < distanceToPrevious and self.index < path.size() - 1):
            index += 1

        return path[self.index] 

<<<<<<< HEAD
=======
=======
        distanceToNext = numpy.linalg.norm(currentState - self.path[self.index])
        distanceToPrevious = numpy.linalg.norm(currentState - self.path[self.index] - 1)
>>>>>>> 5adde3d (updates nextState to nextReference)

        if (distanceToNext < distanceToPrevious and self.index < path.size() - 1):
            index += 1

        return path[self.index] 

<<<<<<< HEAD
>>>>>>> 9f74931 (updates nextState to nextReference)
=======
=======
        distanceToNext = numpy.linalg.norm(currentState - self.path[self.index])
        distanceToPrevious = numpy.linalg.norm(currentState - self.path[self.index] - 1)
>>>>>>> 5adde3d (updates nextState to nextReference)

        if (distanceToNext < distanceToPrevious and self.index < path.size() - 1):
            index += 1

        return path[self.index] 

<<<<<<< HEAD
>>>>>>> 9f74931 (updates nextState to nextReference)
    def getDistance(point1, point2):
        return sqrt((point1.x - point2.x) ** 2 + 
                        (point1.y - point2.y ** 2) + 
                        (point1.z - point2.z) ** 2)

<<<<<<< HEAD
<<<<<<< HEAD
    
>>>>>>> 9f74931 (updates nextState to nextReference)
=======
    #
>>>>>>> 5adde3d (updates nextState to nextReference)
=======
>>>>>>> 766c5d4 (Path follower)
=======
    
>>>>>>> 9f74931 (updates nextState to nextReference)
=======
    #
>>>>>>> 5adde3d (updates nextState to nextReference)
=======
>>>>>>> 766c5d4 (Path follower)
=======
    
>>>>>>> 9f74931 (updates nextState to nextReference)
=======
    #
>>>>>>> 5adde3d (updates nextState to nextReference)
=======
>>>>>>> 766c5d4 (Path follower)
=======
    #The class takes in a State object 'currentState', a Path object 'path', and an int 'index' 
    #which indicates the next point on the path that the sub is intended for. Line 13 forces
    #the index to be 0 if initially given less than 0, and it will be fixed at the max index when it's
    #at the end of path.

    #!!! IS THERE A .SIZE() METHOD FOR PATH TYPE???!!!

class nextReference:
    def __init__ (self, cur_state, path):
        self.currentState = cur_state
        self.path = path
        if len(self.path) == 0:
            self.index = 0
        else:
            self.index = 1
            
    def initializer (self):
        self.path = None
        if len(self.path) == 0:
            self.index = 0
        else:
            self.index = 1
        self.index = 1

    #naive
    def changePath (self, newPath, newIndex):
        self.path = newPath
        if (newIndex < len(newPath)):
            self.index = newIndex
        else:
            self.index = len(newPath) - 1

    def changeState (self, newState):
        print(self.index)
        self.currentState = newState

    #index indicate the current path target
    def getNextReference (self):
        if self.index < 0:
            self.index = 0
            
        if len(self.path) == 1:
            return self.path[self.index]

        else:
            distanceToNext = np.linalg.norm(self.currentState - self.path[self.index])
            distanceToPrevious = np.linalg.norm(self.currentState - self.path[self.index - 1])
            if (distanceToNext < distanceToPrevious and self.index < len(self.path) - 1):
                self.index += 1
        return self.path[self.index] 
            

            
>>>>>>> origin/Deren/controller
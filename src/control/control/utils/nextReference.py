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
        
    def initializer (self):
        self.path = None
        self.index = 0

    #naive
    def changePath (self, newPath, newIndex):
        self.path = newPath
        if (newIndex < newPath.size()):
            self.index = newIndex
        else:
            self.index = newPath.size() - 1

    def changeState (self, newState):
        self.cur_state = newState

    #index indicate the current path target
    def getNextReference (self):
        if index < 0:
            index = 0
        
        distanceToNext = numpy.linalg.norm(currentState - self.path[self.index])
        distanceToPrevious = numpy.linalg.norm(currentState - self.path[self.index] - 1)

        if (distanceToNext < distanceToPrevious and self.index < path.size() - 1):
            index += 1

        return path[self.index] 

    #
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

    #in progress
    def changePath (self, newPath):
        newIndex = self.index
        if newIndex + 1 < newPath.size():
            if newPath[newIndex]:
        self.path = newPath

    def changeState (self, newState):
        self.cur_state = newState

    def getNextReference (self):
        if index < 0:
            index = 0
        
        currentPoint = cur_state.pos
        
        distanceToNext = sqrt((currentState.x - self.path[self.index].x) ** 2 + 
                        (currentState.y - self.path[self.index].y ** 2) + 
                        (currentState.z - self.path[self.index].z) ** 2)

        distanceToPrevious = sqrt((currentState.x - self.path[self.index - 1].x) ** 2 + 
                        (currentState.y - self.path[self.index - 1].y ** 2) + 
                        (currentState.z - self.path[self.index - 1].z) ** 2)
        if distanceToNext < distanceToPrevious and index < path.size() - 1:
            index += 1
        return path[self.index] 

    def getDistance(point1, point2):
        return sqrt((point1.x - point2.x) ** 2 + 
                        (point1.y - point2.y ** 2) + 
                        (point1.z - point2.z) ** 2)

    
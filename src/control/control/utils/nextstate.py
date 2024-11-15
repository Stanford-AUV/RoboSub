import numpy as np

#The class takes in a State object 'currentState', a Path object 'path', and an int 'index' 
#which indicates the next point on the path that the sub is intended for. Line 13 forces
#the index to be 0 if initially given less than 0, and it will be fixed at the max index when it's
#at the end of path.

class nextState:
    def __init__ (self, currentState, path, index):
        self.currentState = currentState
        self.path = path
        self.index = index

    def getNextReference (self):
        if index < 0:
            index = 0
        

        distanceToNext = sqrt((currentState.x - self.path[self.index].x) ** 2 + 
                        (currentState.y - self.path[self.index].y ** 2) + 
                        (currentState.z - self.path[self.index].z) ** 2)

        distanceToPrevious = sqrt((currentState.x - self.path[self.index - 1].x) ** 2 + 
                        (currentState.y - self.path[self.index - 1].y ** 2) + 
                        (currentState.z - self.path[self.index - 1].z) ** 2)
        if distanceToNext < distanceToPrevious and index < path.size() - 1:
            index += 1
        return path[self.index] 
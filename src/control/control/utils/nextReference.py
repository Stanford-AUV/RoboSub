import numpy as np

    #The class takes in a State object 'currentState', a Path object 'path', and an int 'index' 
    #which indicates the next point on the path that the sub is intended for. Line 13 forces
    #the index to be 0 if initially given less than 0, and it will be fixed at the max index when it's
    #at the end of path.

    #!!! IS THERE A .SIZE() METHOD FOR PATH TYPE???!!!

    class nextReference:
        def __init__ (self, cur_state, path):
            self.currentState = cur_state
            self.path = path
            if self.path.shape[0] == 0:
                self.index = 0
            else:
                self.index = 1
            
        def initializer (self):
            self.path = None
            self.index = 1

        #naive
        def changePath (self, newPath, newIndex):
            self.path = newPath
            if (newIndex < newPath.shape[0]):
                self.index = newIndex
            else:
                self.index = newPath.shape[0] - 1

        def changeState (self, newState):
            print(self.index)
            self.currentState = newState

        #index indicate the current path target
        def getNextReference (self):
            if self.index < 0:
                self.index = 0
            
            if self.path.shape[0] == 1:
                return self.path[self.index]

            else:
                distanceToNext = np.linalg.norm(self.currentState - self.path[self.index])
                distanceToPrevious = np.linalg.norm(self.currentState - self.path[self.index - 1])
                if (distanceToNext < distanceToPrevious and self.index < self.path.shape[0] - 1):
                    self.index += 1
            return self.path[self.index] 
            

            

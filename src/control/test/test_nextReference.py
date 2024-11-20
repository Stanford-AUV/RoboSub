import pytest
import numpy as np
import control.utils.nextReference

def test_nextReference():

    curState = np.array([-3, -1, -2])
    testPath = np.array([-2, -1, -2], [0, 0, 0], [1, 1, 1])
    referencePoint = nextReference(curState, testPath)
    
    assert referencePoint.index == 0

    changeState(np.array([-2, -1, -2])) 
    assert referencePoint.index = 1
    
    changeState(np.array([-2, 0, 3]))
    assert referencePoint.index == 1

    changeState(np.array([0, 0, 1]))
    assert referencePoint.index == 2

    changeState(np.array([100, 100, 100]))
    assert referencePoint.index == 2

    changeState(np.array([-100, -100, -100]))
    assert referencePoint.index == 2
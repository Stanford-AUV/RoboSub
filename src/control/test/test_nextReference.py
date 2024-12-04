import pytest
import numpy as np
from control.utils.nextReference import nextReference


def test_nextReference():

    curState = np.array([-3, -1, -2])
    testPath = [
        np.array([-3, -1, -2]),
        np.array([-2, -1, -2]),
        np.array([0, 0, 0]),
        np.array([1, 1, 1]),
    ]
    referencePoint = nextReference(curState, testPath)

    print(1)
    print(referencePoint.index)
    assert referencePoint.index == 0

    print(referencePoint.index)
    referencePoint.changeState(np.array([-2, -1, -2]))
    assert referencePoint.index == 1

    print(referencePoint.index)
    referencePoint.changeState(np.array([-2, 0, 3]))
    assert referencePoint.index == 1

    print(referencePoint.index)
    referencePoint.changeState(np.array([0, 0, 1]))
    assert referencePoint.index == 2

    print(referencePoint.index)
    referencePoint.changeState(np.array([100, 100, 100]))
    assert referencePoint.index == 2

    print(referencePoint.index)
    referencePoint.changeState(np.array([-100, -100, -100]))
    assert referencePoint.index == 2

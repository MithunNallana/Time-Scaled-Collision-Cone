import numpy as np

from tscc.optimization.computeconstraints import computeconstraints
from tscc.optimization.computescale import computescale
from tscc.utils.scalegrid import scalegrid


def timescaling(stateRobo, radiusRobo, limitsRobo, stateObst, radiusObst,
                deltaT, deltaTSmall):

    # initial guess scale
    guessScale = 1.5

    # iteration for convexification of constraint
    iters = 3

    # weight given to constraint violation slack terms
    weightSlack = 1000

    # optimizing for scale
    a, b, c = computeconstraints(stateObst, stateRobo, radiusObst, 
                                 radiusRobo, deltaT)
    scale = computescale(a, b, c, iters, stateRobo, limitsRobo, 
                         weightSlack, guessScale, deltaT)
    scaleGrid = scalegrid(scale, deltaT, deltaTSmall)

    nScale = len(scaleGrid)
    velRoboNew = np.zeros(nScale)
    omegaRoboNew = np.zeros(nScale)
    for m in range(nScale):
        velRoboNew[m] = stateRobo[3]*scaleGrid[m]
        omegaRoboNew[m] = stateRobo[4]*scaleGrid[m]

    return velRoboNew, omegaRoboNew

import numpy as np


# state update considering simple differential drive motion model
def updatestate(currentX, currentY, currentTheta,
                currentVel, currentOmega, deltaT):

    nextTheta = currentTheta + currentOmega*deltaT
    nextX = currentX + currentVel*np.cos(nextTheta)*deltaT
    nextY = currentY + currentVel*np.sin(nextTheta)*deltaT

    return nextX, nextY, nextTheta

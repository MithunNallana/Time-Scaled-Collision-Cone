from tscc.model.updatestate import updatestate


def computecoefficients(stateObst, stateRobo, radiusObst, radiusRobo, deltaT):
    ''' Compute collision cone constraint coefficients '''
    clearanceR = 0.0
    R = radiusObst + radiusRobo + clearanceR

    # unrolling state and control data of robot and obstacle
    xRobo = stateRobo[0]
    yRobo = stateRobo[1]
    thetaRobo = stateRobo[2]
    velRobo = stateRobo[3]
    omegaRobo = stateRobo[4]

    xObst = stateObst[0]
    yObst = stateObst[1]
    thetaObst = stateObst[2]
    velObst = stateObst[3]

    # updating state of robot and obstacle
    xRoboNxt, yRoboNxt, _ = updatestate(xRobo, yRobo, thetaRobo, velRobo,
                                        omegaRobo, deltaT)
    xdotRobo = (xRoboNxt-xRobo)/deltaT
    ydotRobo = (yRoboNxt-yRobo)/deltaT

    xObstNxt, yObstNxt, _ = updatestate(xObst, yObst, thetaObst, 
                                        velObst, 0, deltaT)
    xdotObst = (xObstNxt-xObst)/deltaT
    ydotObst = (yObstNxt-yObst)/deltaT

    if ((xRoboNxt - xObstNxt)**2 + (yRoboNxt - yObstNxt)**2 < R**2):
        raise ValueError("Your lookahead is already in collision. \
                          Try reducing deltaT")

    # coefficients of quadratic collision cone constraints
    # detailed in additional_material.pdf
    m = (xdotRobo*(xRoboNxt-xObstNxt)) + (ydotRobo*(yRoboNxt-yObstNxt))
    n = (xdotObst*(xObstNxt-xRoboNxt)) + (ydotObst*(yObstNxt-yRoboNxt))

    u = -(xRoboNxt-xObstNxt)**2 - (yRoboNxt-yObstNxt)**2 + R**2
    p = xdotRobo**2 + ydotRobo**2
    q = -2*(xdotRobo*xdotObst + ydotRobo*ydotObst)
    r = xdotObst**2 + ydotObst**2

    A = m**2 + (u*p)
    B = 2*m*n + (u*q)
    C = n**2 + (u*r)

    return A, B, C


def computeconstraints(stateObst, stateRobo, radiusObst, radiusRobo, deltaT):
    ''' compute the a,b and c coefficients of quadratic collision
    cone equation'''
    a = []
    b = []
    c = []
    nObst = len(stateObst)

    for i in range(nObst):
        tempa, tempb, tempc = computecoefficients(stateObst[i], stateRobo,
                                                  radiusObst[i], radiusRobo,
                                                  deltaT)
        a.append(tempa)
        b.append(tempb)
        c.append(tempc) 

    return a, b, c

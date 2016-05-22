
# initial uniform probability distribution or all particles
#p = [0.2, 0.2, 0.2, 0.2, 0.2]
p = [1, 0, 0, 0, 0]

# actual values in the world
world = ['green', 'red', 'red', 'green', 'green']

# what we are sensing for
measurements = ['red']
motions = [1]


pHit = 0.6 # probability that it was sensed correctly
pMiss = 0.2 # probability that it was sensed incorrectly

pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1

# function that moves to a given number of cells; it basically shifts previous probabilities
def move(p, numberOfCells):

    q = []
    for i in range(len(p)):
        # account for inexact movement with probability of undershooting/overshooting
        cellIndex = i - numberOfCells
        #print cellIndex
        arraySize = len(p)
        s = pExact * p[( cellIndex ) % arraySize]
        print p[( cellIndex ) % arraySize]
        print s
        s = s + pOvershoot * p[(cellIndex - 1) % arraySize]
        print p[( cellIndex - 1 ) % arraySize]
        print s
        s = s + pUndershoot * p[(cellIndex + 1) % arraySize]
        print p[( cellIndex + 1 ) % arraySize]
        print s
        print "--------"
        q.append( s )
    #print q
    return q


# function that samples the environment
def sense(p, Z):
    q = []

    # go through all particles and update probabilities (calculate the non-normalized posterior P)
    for i in range(len(p)):

        # whether or not Z was thought to be found in the real world
        hit = (Z == world[i])

        if hit:
            updatedProbability = p[i] * pHit
        else:
            updatedProbability = p[i] * pMiss

        q.append(updatedProbability)

    #print q

    normalizingConstant = sum(q) # calculate the normalizing constant

    # normalize the P distribution
    for i in range(len(p)):
        q[i] =  q[i] / normalizingConstant

    return q


for i in range(len(measurements)):
    p = sense(p, measurements[i])
    print p

    p = move(p, motions[i])
    print p


#print p[-1]


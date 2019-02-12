from math import *
import sys
import warnings

## state variables named assuming positive acceleration

POSJERK = 0
NOJERK = 1
NEGJERK = 2
END = 3

def vdiff(u, v):
    r = []
    for i in range(len(u)):
        r.append(u[i] - v[i])
    return r
def vsum(u, v):
    r = []
    for i in range(len(u)):
        r.append(u[i] + v[i])
    return r
def smult(s, u):
    r = []
    for i in range(len(u)):
        r.append(s*u[i])
    return r

def accelprofile(v0, v1):
    if v0 == v1:
        return [[v0, 0, 0]]
    accel = 1
    if v1 < v0:
        accel = -1

    ## Do we have time to reach amax?

    deltav = abs(v1-v0)
    T = round(sqrt(deltav/jmax))
    A = jmax*T
    if A < amax:
        t1 = int(T)
        t2 = t1
        t3 = int(2*T)

    else:
        t1 = int( ceil(amax/jmax)) 
        t2 = int( t1 + abs(v1-v0)/amax - amax/jmax) 
        t3 = t2 + t1

    t = 0
    v = v0
    a = 0
    j = 0

    trajectory = [[v,a,j]]
    jm = jmax


    state = POSJERK

    while state != END:
        t += 1
        if state == POSJERK:
            if t > t1:
                if t1 == t2:
                    state = NEGJERK
                    T = abs(a/float(jm))
                    jm = abs(a/ceil(T))

                    j = -accel * jm
                    a += j
                    v += a
                    print T, jm, jmax
                else:
                    state = NOJERK
                    j = 0
                    a = accel * amax
                    v += a
            else:
                j = accel * jm
                a += j
                if abs(a) > amax:
                    a = accel * amax
                v += a
        elif state == NOJERK:
            if t > t2:
                state = NEGJERK
                j = -accel * jm
                a += j
                v += a
            else:
                j = 0
                a += j
                v += a
        elif state == NEGJERK:
            j = -accel*jm
            a += j
            v += a
            if accel * a < 0 or accel * (abs(v)- abs(v1)) > 0:
                j = 0
                a = 0
                v = v1
                state = END

        trajectory.append([v,a,j])
    return trajectory

def evaluateBezier(P, t):
    term = smult((1-t)**3, P[0])
    term = vsum(term, smult(3*t*(1-t)**2, P[1]))
    term = vsum(term, smult(3*t*t*(1-t), P[2]))
    return vsum(term, smult(t**3, P[3]))

def evaluateBezierPrime(P, t):
    term = smult(-3*(1-t)**2, P[0])
    term = vsum(term, smult(9*t*t-12*t+3, P[1]))
    term = vsum(term, smult(6*t-9*t*t, P[2]))
    return vsum(term, smult(3*t*t, P[3]))

def length(v):
    r = 0
    for p in v:
        r += p*p
    return sqrt(r)

def theta(v):
    return atan2(v[1], v[0])

def speed(P, t):
    return length(evaluateBezierPrime(P,t))

def angle(P, t):
    v = evaluateBezierPrime(P,t)
    return theta(v)

def multiplyrow(m, j, s):
    for k in range(len(m[j])):
        m[j][k] *= s
        
def rowreplace(m, j, k):
    mult = -m[k][j]
    for i in range(j, len(m[k])):
        m[k][i] += mult*m[j][i]

def findP1(K, comp):
    N = len(K)-1
    matrix = []
    for i in range(N):
        matrix.append([0.0]*N)
    matrix[0][0] = 2.0
    matrix[0][1] = 1.0
    matrix[0].append(K[0][comp] + 2.0*K[1][comp])

    for i in range(1, N-1):
        matrix[i][i-1] = 1.0
        matrix[i][i] = 4.0
        matrix[i][i+1] = 1.0
        matrix[i].append(4.0*K[i][comp] + 2.0*K[i+1][comp])

    matrix[N-1][N-2] = 2.0
    matrix[N-1][N-1] = 7.0
    matrix[N-1].append(8.0*K[N-1][comp] + K[N][comp])

    ## forward elimination
    for i in range(N):
        multiplyrow(matrix, i, 1/float(matrix[i][i]))
        if i < N-1: rowreplace(matrix, i, i+1)

    for i in range(N-1, 0, -1):
        rowreplace(matrix, i, i-1)

    P1 = []
    for i in range(N):
        P1.append(matrix[i][N])
    return P1

def findP2(K, P1, comp):
    N = len(K) -1
    P2 = []
    for i in range(N-1):
        P2.append(2.0*K[i+1][comp] - P1[i+1])
    P2.append((K[N][comp] + P1[N-1])/2.0)
    return P2
    

def buildtrajectory(K):
    P1x = findP1(K, 0)
    P1y = findP1(K, 1)
    P2x = findP2(K, P1x, 0)
    P2y = findP2(K, P1y, 1)

    beziers = []
    for i in range(len(K)-1):
        curve = [K[i], [P1x[i], P1y[i]], [P2x[i], P2y[i]], K[i+1]]
        beziers.append(curve)
    return beziers

def findDecelPoint(bezier, velocity):
    b = bezier[:]
    b.reverse()

    accel = accelprofile(velocity[2], velocity[1])
    x = 0

    for i in range(len(accel)-1):
        v0 = accel[i][0]
        v1 = accel[i+1][0]
        distance = (v0+v1)/2.0

        speedx = speed(b,x)
        deltax = distance/speedx
        x1 = x + deltax
        x += 2 * distance/ (speedx + speed(b, x1))

    return 1-x

def takestep(b, x, v0, v1, radius, left, right, heading):
    distance = (v0+v1)/2.0

    speedx = speed(b, x)
    deltax = distance/speedx
    xp = x+deltax
    x1 = x + 2 * distance / (speedx + speed(b, xp))

    dangle = angle(b, x1) - angle(b, x)
    if dangle > pi:
        dangle -= 2*pi
    if dangle < -pi:
        dangle += 2*pi

    rtheta = radius*dangle

    leftdistance = distance - rtheta
    rightdistance = distance + rtheta
    left.append([left[-1][0] + leftdistance,
                 2*leftdistance - left[-1][1]])
    right.append([right[-1][0] + rightdistance,
                  2*rightdistance - right[-1][1]])
    heading.append(angle(b, x1)*180/pi)
    
    return x1

def backonestep(left, right, heading):
    left.pop(-1)
    right.pop(-1)
    heading.pop(-1)

def buildprofile(beziers, commands, velocities, wheelbase):
    global commandinsert
    radius = wheelbase/2.0

    ## position, velocity
    
    left = [[0,0]]
    right = [[0,0]]
    heading = [0]

    N = len(beziers)
    leftover = 0
    for j in range(N):
        bezier = beziers[j]
        velocity = velocities[j]
        xdecel = findDecelPoint(bezier, velocity)
        cmd = commands[j]
        if cmd != emptycmd:
            commandinsert.append([len(left)+1, cmd])

#        x = leftover/speed(bezier, 0)
        x = leftover

        accel = accelprofile(velocity[0], velocity[1])
        for i in range(len(accel)-1):
            v0 = accel[i][0]
            v1 = accel[i+1][0]
            lastx = x
            x = takestep(bezier, x, v0, v1, radius, left, right, heading)

        if x > xdecel:
            print "Not enough time to implement change in speed in curve", j
            sys.exit(1)
                                 
        while x < xdecel:
            lastx = x
            x = takestep(bezier, x, velocity[1], velocity[1], radius,
                         left, right, heading)

        if (lastx+x)/2.0 > xdecel:
            x = lastx
            backonestep(left, right, heading)

        accel = accelprofile(velocity[1], velocity[2])
        for i in range(len(accel)-1):
            v0 = accel[i][0]
            v1 = accel[i+1][0]
            lastx = x
            x = takestep(bezier, x, v0, v1, radius, left, right, heading)

        if (lastx + x)/2.0 > 1:
            leftover = 1-lastx
            backonestep(left, right, heading)
        else:
            leftover = x - 1
    return [left, right, heading]

def outputprofile(filename, left, right, heading, commandinsert):
    out = open(filename, "w")
    out.write("%5.4f\n" % vmax)
    linecount = 0
    while len(left) > 0:
        l = left.pop(0)
        r = right.pop(0)
        h = heading.pop(0) - initHeading
        if len(commandinsert) > 0 and linecount == commandinsert[0][0]:
            cmd = commandinsert[0][1]
            commandinsert.pop(0)
        else:
            cmd = emptycmd
        s = "%5.4f,%5.4f,%5.4f,%5.4f,%5.2f,%s\n" % ( l[0], l[1], r[0], r[1], h, cmd )
        out.write(s)
        linecount += 1

## physical units on the field

#fps = 8.0         ## feet/sec (stronghold)
fps = 12.0         ## steamworks
fps = 8.4

fps = 10.0 ## for the spark robot
fps = 12.0 ## for power up
vmax = fps * 12    ## inches/sec
amax = vmax * 1.0  ## inches/sec/sec (reaches vmax in 1/1th seconds)
jmax = amax * 10.0 ## inches/sec/sec (reaches amax in 1/10th seconds)

## units per cycle
##   time measured in cycles from this point
deltat = 0.01   ## 10 ms/cycle
vmax *= deltat
amax *= deltat**2
jmax *= deltat**3

initHeading = 0
sys.argv = [0, 'left-left-rocket-back.csv']
#Get file command line argument and store it to CsvPath
if len(sys.argv) < 2:
    print "Usage:  python profile.py filename"
    sys.exit(1)

readfile = sys.argv[1]
print "Reading", readfile
input = open(readfile)
lines = input.readlines()
print lines
input.close()

K = []
commands = []
emptycmd = " "
commandinsert = []
for line in lines:
    line = line.rstrip()
    splitline = line.split(",")
    K.append([float(splitline[0]),
              float(splitline[1])])
    if len(splitline) == 2:
        commands.append(emptycmd)
    else:
        commands.append(splitline[2])

velocityfile = readfile[:-3] + "velocities.csv"
input = open(velocityfile)
lines = input.readlines()
velocities = []
for line in lines:
    line = line.rstrip()
    splitline = line.split(",")
    for i in range(3):
        # modify this line below:  take out 12/float(fps)
        splitline[i] = float(splitline[i]) * vmax # * 12.0/float(fps)
    velocities.append(splitline)
print velocities
input.close()

if len(K) - len(velocities) != 1:
    print "The number of velocities is not correct"
    sys.exit(1)

wheelbase = 24
beziers = buildtrajectory(K)
left, right, heading = buildprofile(beziers, commands, velocities,
                                    wheelbase)

left[-1][1] = 0.0
right[-1][1] = 0.0

print "steps =", len(left)

outfile = readfile[:-4] + ".profile.csv"
print "Writing", outfile
outputprofile(outfile, left, right, heading, commandinsert)

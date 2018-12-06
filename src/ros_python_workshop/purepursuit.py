#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time, sys, math


odometry_pose = None

def makeWaypoint (x, y):
    return { 'x': x,
             'y': y
    }

def makePath (X, Y):
    path = []

    # If sizes mismatch, return empty list
    if len(X) != len(Y):
        return path

    
    for i in range(len(X)):
        path.append(makeWaypoint(X[i], Y[i]))
    return path

def makePoint2D(x, y, theta):
    return { 'x': x,
             'y': y,
             'theta': theta
    }

def odom2world(startPosWorld, currentPosOdom ):
    # The turtlebot's odometer is initialized at [0, 0, 0, 0, 0, 0].
    # We need it to be relative to the actual map.
    # So we have to know what that [0, 0, 0, 0, 0, 0] actually is
    # startPosWorld: robot's starting world position 
    # currentPosOdom: robot's current odometry position

    if (currentPosOdom == None):
        return None

    currentPosWorld = {
    
        "position": { "x": startPosWorld["position"]["x"] + 
                          currentPosOdom.position.x,
                      "y": startPosWorld["position"]["y"] + 
                          currentPosOdom.position.y,
                      "z": startPosWorld["position"]["z"] + 
                          currentPosOdom.position.z, 
        },

        "orientation": { "x": startPosWorld["orientation"]["x"] + 
                             currentPosOdom.orientation.x,
                         "y": startPosWorld["orientation"]["y"] + 
                             currentPosOdom.orientation.y,
                         "z": startPosWorld["orientation"]["z"] + 
                             currentPosOdom.orientation.z,
                         "w": startPosWorld["orientation"]["w"] + 
                             currentPosOdom.orientation.w,
        },
    }
    

    return currentPosWorld

def printPoseDict(pose):
    print ("Position")
    print ("\t x:"), 
    print (pose["position"]["x"])
    print ("\t y:"), 
    print (pose["position"]["y"])
    print ("\t z:"), 
    print (pose["position"]["z"])

    print ("Orientation")
    print ("\t x:"), 
    print (pose["orientation"]["x"])
    print ("\t y:"), 
    print (pose["orientation"]["y"])
    print ("\t z:"), 
    print (pose["orientation"]["z"])
    print ("\t w:"), 
    print (pose["orientation"]["w"])

def printPose2D (pose):
    print ("(x: %d, y: %d, %.3f" \
        % (pose["x"], pose["y"], pose["theta"]))

# Start ROS Functions

class turtlebot_velocity (object): 

    _pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size = 10)

    def __init__(self):
        rospy.init_node('turtlebot_trajectoryFollow')
        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size = 10)
        rospy.sleep(1)

    def write(self, linear, angular):
        self._pub.publish(Twist(Vector3(linear, 0, 0),
            Vector3(0, 0, angular)))

def turtlebot_odometry_callback (msg):
    global odometry_pose
    odometry_pose = msg.pose.pose

def listener():
    return rospy.Subscriber("odom", Odometry, turtlebot_odometry_callback)

# End ROS Functions

# Start PurePursuit function

def dot2 (v, u):
    return v['x'] * u['x'] + v['y'] * u['y']

def dist (a, b):
    return pow ( \
        (b['x'] - a['x']) * (b['x'] - a['x']) + \
        (b['y'] - a['y']) * (b['y'] - a['y']), 
    0.5)

def distP2S (currentPosition, waypointA, waypointB):
    # Obtains difference between current position and segment s
    # Returns the shortest distance from position to segment, and the point
    # of the segment that gives the shortest distance

    c1 = 0.0
    c2 = 0.0
    di = 0.0
    b = 0.0

    waypointV = makeWaypoint(waypointB['x'] - waypointA['x'], 
                             waypointB['y'] - waypointA['y'])
    waypointW = makeWaypoint(currentPosition['x'] - waypointA['x'], 
                             currentPosition['y'] - waypointA['y'])
    waypointRes = makeWaypoint(-1, -1)

    c1 = dot2 (waypointW, waypointV)

    if (c1 <= 0.0):
        di = dist(currentPosition, waypointA)
        waypointRes = makeWaypoint(waypointA['x'], waypointA['y'])
        return waypointRes, di
        
    if (c2 <= c1):
        di = dist(currentPosition, waypointB)
        waypointRes = makeWaypoint(waypointB['x'], waypointB['y'])
        return waypointRes, di
        
    b = c1 / c2
    waypointRes = makeWaypoint(wayPointA['x'] + b * waypointV['x'],
                               wayPointA['y'] + b * waypointV['y'])
    di = dist (currentPosition, waypointRes)
    return waypointRes, di
    

def pp_getPoint(pose, path, currentIdx, lookAhead):
    # Returns a point on the path that is at the lookAhead distance

    closestIdx = 0

    target = { "x": -1, "y": -1 }
    potentialTarget = { "x": -1, "y": -1 }
    targetPosition = { "x": None, "y": None, "theta": None }

    dmin = float("inf")
    d = 0.0
    d1 = 0.0
    d2 = 0.0

    # Number of waypoints
    numWaypoints = len(path)

    # Init segment as length of path
    segments = [0.0 for i in range(numWaypoints)]

    '''
    Part 1: Find closest segment, and closest point on segment
    '''

    # From the current waypoint to final waypoint,
    for i in range(currentIdx, numWaypoints - 1):

        # Length of segment
        segments[i] = dist (path[i], path[i + 1])

        # Find closest point on segment
        potentialTarget, d = distP2S(pose, path[i],  path[i + 1])

        # Check if closest segment
        if d < dmin:
            target["x"] = potentialTarget["x"]
            target["y"] = potentialTarget["y"]
            dmin = d
            closestIdx = i

    # Check if waypoint was changed
    if currentIdx != closestIdx:
        currentIdx = closestIdx
        #if closestIdx == numWaypoints - 2 # If is penultimate waypoint
            

    '''
    Part 2: Find segment ahead in lookahead distance
    '''
    
    d1 = dist (target, path[closestIdx + 1]) # MIGHT be 'currentIdx'
    
    # Index of lookahead point segment
    k = closestIdx
    while d1 < lookAhead and k + 1 < numWaypoints - 1:
        # Searched point on this segment
        k = k + 1
        d1 = d1 + segments[k]
    

    '''
    Part 3: Obtain t parameter in the segment
    '''

    d2 = d1 - lookAhead
    t = (segments[k] - d2) / segments[k]

    # Obtain point with t parameter
    targetPosition['x'] = path[k]['x'] + \
                          (path[k + 1]['x'] - path[k]['x']) * t
    targetPosition['y'] = path[k]['y'] + \
                          (path[k + 1]['y'] - path[k]['y']) * t

    angleSegment = math.atan2(path[k + 1]['y'] - path[k]['y'],
                         path[k + 1]['x'] - path[k]['x'])

    targetPosition['theta'] = angleSegment
    

    return targetPosition, k


# End PurePursuit functions


def main ():

    ###########
    # Options #
    ###########

    startPosWorld = {
        "position": { "x": 150,
                      "y": 200,
                      "z": 2.01, 
        },
        "orientation": { "x": 0,
                         "y": 0,
                         "z": 0,
                         "w": 0,
        },
    }
 
    roll, pitch, yaw = euler_from_quaternion ( [\
            startPosWorld['orientation']['x'],
            startPosWorld['orientation']['y'],
            startPosWorld['orientation']['z'],
            startPosWorld['orientation']['w'] ])
    worldPose2D = makePoint2D (startPosWorld["position"]["x"], 
                               startPosWorld["position"]["y"], 
                               yaw)

    Xwaypoints = [startPosWorld['position']['x'], 170.0, 200.0, 210.0]
    Ywaypoints = [startPosWorld['position']['y'], 210.0, 160.0, 160.0]
    path = makePath(Xwaypoints, Ywaypoints)

    minDist2Goal = 5;

    # Pure Pursuit
    dist_wheel_to_center = 0.005
    k = 0.1
    lookAhead = 1
    Kp = 1.0
    L = 2.9 
    
    linearSpeed = 0.1

    #########
    # SETUP #
    #########

    # Start timer (for logging, etc)
    startTime = time.time()

    # Give ROS a chance to start up
    rospy.sleep (1)

    # halt boolean to control when to stop tracking
    halt = False

    # Initiallize velocity controller
    velctrl = turtlebot_velocity()

    # Start listening to odometer
    sub = listener();

    # Set goal location as final point in path
    goal = path[len(path) - 1]

    # Set initial distance to goal as max,
    # since it is yet unmeasured.
    # (Don't want it set to 0 since that would signal arrival)
    dist2Goal = float("inf")

    waypoint = None

    pathIdx = 0

    # Counter for # times the main follower
    # loop has iterated. Used to control how often the
    # visualization/log messages display.
    iterationCount = 0



    # Start PurePursuit loop
    while (halt == False):
        rospy.sleep(0.05)

        #########
        # SENSE #
        #########

        # Get robot current position
        if (odometry_pose == None):
            continue
        worldPose = odom2world (startPosWorld, odometry_pose)
        roll, pitch, yaw = euler_from_quaternion ( [\
            worldPose['orientation']['x'],
            worldPose['orientation']['y'],
            worldPose['orientation']['z'],
            worldPose['orientation']['w'] ])
        worldPose2D = makePoint2D (worldPose["position"]["x"], 
                                   worldPose["position"]["y"], 
                                   yaw)
        ###########
        # PROCESS #
        ###########

        
        # Use pure pursuit to get angle to move toward goal
        targetPoint, pathIdx = pp_getPoint(worldPose['position'], path, pathIdx, lookAhead)

        dist2target = dist (worldPose['position'], targetPoint)
        dist2goal = dist (worldPose['position'], goal)
        if (dist2goal < minDist2Goal):
            halt = True
            continue

        theta = yaw
        # Calculate curvature
        dx = worldPose['position']['x'] - targetPoint['x']
        dy = worldPose['position']['y'] - targetPoint['y']
        x1 = math.cos(theta) * dx + math.sin(theta) * dy 
        y1 = -1 * math.sin(theta) * dx + math.cos(theta) * dy

        if (x1 * x1 + y1 * y1) == 0:
            curv = 0
        else:
            curv = (2.0 / (x1 * x1 + y1 * y1)) * (-1 * y1)

        iterationCount = iterationCount + 1

        ###########
        # CONTROL #
        ###########

        # Send movement control to robot
        velctrl.write(linearSpeed, curv * linearSpeed)

        #############
        # VISUALIZE #
        #############

        visTime = time.time()
        runTime = visTime - startTime
        if (iterationCount % 100 == 0):
            print ("")
            #printPoseDict (worldPose)
            print ("Robot position: (x: %d, y: %d, theta: %.3f)" \
                % (worldPose["position"]["x"], worldPose["position"]["y"], theta))
            print ("Next waypoint: (x: %d, y: %d, segment: %d)" \
                % (targetPoint['x'], targetPoint['y'], pathIdx))
            print ("Distance to: (waypoint: %.3f, goal: %.3f)" \
                % (dist2target, dist2goal))
            print ("Time traveled: %.3f s" \
                % (runTime))
            print ("")

    ##########
    # FINISH #
    ##########
    
    # Halt robot
    velctrl.write(0, 0)

    endTime = time.time()
    runTime = endTime - startTime
    print ("Goal reached in: ", runTime, "seconds")


if __name__ == '__main__':
    main()







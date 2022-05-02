#This is where your main robot code resides. It extendeds from the BrickPi Interface File
#It includes all the code inside brickpiinterface. The CurrentCommand and CurrentRoutine are important because they can keep track of robot functions and commands. Remember Flask is using Threading (e.g. more than once process which can confuse the robot)
from interfaces.brickpiinterface import *
import grid
import global_vars as GLOBALS
import logging, math
import numpy as np

"""
    Proposed Algorithm:
        1. turn right
            1a. 
"""

class Robot(BrickPiInterface):
    
    def __init__(self, timelimit=10, logger=logging.getLogger(), position=(0,0)):
        super().__init__(timelimit, logger, position)
        self.CurrentCommand = "stop" #use this to stop or start functions
        self.CurrentRoutine = "stop" #use this stop or start routines
        self.FoundWalls = {}
        return
        
    def calibrate_position(self, squaresize=30):
        xpos = (squaresize/2)-self.get_ultra_sensor()
        self.rotate_power_degrees_IMU(15,-90,11.0)
        ypos = (squaresize/2)-self.get_ultra_sensor()
        self.rotate_power_degrees_IMU(15,90,11.0)
        self.position=(xpos/squaresize,ypos/squaresize)
        return self.position
    
    def move_distance_encoder(self,distanceCm,direction=1,speed=150,power=300):
        distance = (direction*distanceCm*360)/(np.pi*5.6)
        BP = self.BP
        try:
            self.CurrentCommand = "move_distance_encoder"
            symbol = '<'
            if distance == 0:
                return
            elif distance < 0:
                symbol = '<='
            else:
                symbol = '>='
            BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A)) # reset encoder A
            BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D)) # reset encoder D
            BP.set_motor_limits(BP.PORT_A, power, speed)    # float motor D
            BP.set_motor_limits(BP.PORT_D, power, speed)          # optionally set a power limit (in percent) and a speed limit (in Degrees Per Second)
            while self.CurrentCommand == "move_distance_encoder":
                BP.set_motor_position(BP.PORT_D, distance+10)    # set motor A's target position to the current position of motor D
                BP.set_motor_position(BP.PORT_A, distance+10)
                time.sleep(0.02)
                if eval("BP.get_motor_encoder(BP.PORT_D)" + symbol + "distance") or eval("BP.get_motor_encoder(BP.PORT_A)" + symbol + "distance"):
                    break
                #print("A:  " + str(distance+10) + "   " + str(BP.get_motor_encoder(BP.PORT_A)))
                #print("D:  " + str(distance+10) + "   " + str(BP.get_motor_encoder(BP.PORT_D)))
            return (((BP.get_motor_encoder(BP.PORT_A)+BP.get_motor_encoder(BP.PORT_D))/2)*(np.pi*5.6))/(direction*360)
        except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
            BP.reset_all()
        return

    def move_encoder(self,direction=1,speed=150,power=300):
        BP = self.BP
        try:
            self.CurrentCommand = "move_encoder"
            BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A)) # reset encoder A
            BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D)) # reset encoder D
            BP.set_motor_limits(BP.PORT_A, power, speed)    # float motor D
            BP.set_motor_limits(BP.PORT_D, power, speed)          # optionally set a power limit (in percent) and a speed limit (in Degrees Per Second)
            while self.CurrentCommand == "move_encoder":
                BP.set_motor_position(BP.PORT_D, BP.get_motor_encoder(BP.PORT_A)+(direction*5))    # set motor A's target position to the current position of motor D
                BP.set_motor_position(BP.PORT_A, BP.get_motor_encoder(BP.PORT_D)+(direction*5))
                time.sleep(0.02)
            return (((BP.get_motor_encoder(BP.PORT_A)+BP.get_motor_encoder(BP.PORT_D))/2)*(np.pi*5.6))/(direction*360)
        except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
            BP.reset_all()
        return

    def turnLeft(self,angle,speed=100,power=100):   #power percent, degrees/second, degrees
        BP = self.BP
        degrees = angle*2-2
        try:
            BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A)) # reset encoder A
            BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D)) # reset encoder D
            BP.set_motor_limits(BP.PORT_A, -1*power, speed)    # float motor D
            BP.set_motor_limits(BP.PORT_D, power, speed)          # optionally set a power limit (in percent) and a speed limit (in Degrees Per Second)
            while True:
                BP.set_motor_position(BP.PORT_D, degrees+5)    # set motor A's target position to the current position of motor D
                BP.set_motor_position(BP.PORT_A, -1*degrees-5)
                time.sleep(0.02)
                if BP.get_motor_encoder(BP.PORT_D) >= degrees or BP.get_motor_encoder(BP.PORT_A) <= -1*degrees:
                    break
                #print("A:  " + str(-1*degrees+10) + "   " + str(BP.get_motor_encoder(BP.PORT_A)))
                #print("D:  " + str(degrees-10) + "   " + str(BP.get_motor_encoder(BP.PORT_D)))
        except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
            BP.reset_all()
        return 

    def turnRight(self,angle,speed=100,power=100):
        BP = self.BP
        degrees = angle*2+5
        try:
            BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A)) # reset encoder A
            BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D)) # reset encoder D
            BP.set_motor_limits(BP.PORT_D, -1*power, speed)    # float motor D
            BP.set_motor_limits(BP.PORT_A, power, speed)          # optionally set a power limit (in percent) and a speed limit (in Degrees Per Second)
            while True:
                BP.set_motor_position(BP.PORT_A, degrees+10)    # set motor A's target position to the current position of motor D
                BP.set_motor_position(BP.PORT_D, -1*degrees-10)
                time.sleep(0.02)
                if BP.get_motor_encoder(BP.PORT_A) >= degrees or BP.get_motor_encoder(BP.PORT_D) <= -1*degrees:
                    break
                #print("D:  " + str(-1*degrees-10) + "   " + str(BP.get_motor_encoder(BP.PORT_D)))
                #print("A:  " + str(degrees+10) + "   " + str(BP.get_motor_encoder(BP.PORT_A)))
        except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
            BP.reset_all()
        return
 
    def get_new_position(self):
        pass    

    def scan(self, power, degrees, squaresize=30, marginoferror=3, robotposition=(0,0)):
        self.log("BEGIN SCAN")
        # Turning set up (from brickpiinterface)
        bp = self.BP
        if (self.config['imu'] >= SensorStatus.DISABLED):
            return
        self.interrupt_previous_command()
        self.CurrentCommand = "rotate_power_degrees_IMU"
        
        data = {'rotated':0,'elapsed':0}

        symbol = '<'; limit = 0
        if degrees == 0:
            return
        elif degrees < 0:
            symbol = '>='; limit = degrees+marginoferror
        else:
            symbol = '<='; limit = degrees-marginoferror; power = -power
        totaldegreesrotated = 0; lastrun = 0; ultrareading = 0; scanpoints = []
        
        starttime = time.time()
        #start motors 
        bp.set_motor_power(self.rightmotor, power)
        bp.set_motor_power(self.leftmotor, -power)

        # Scan in for degrees
        while eval("totaldegreesrotated" + str(symbol) + "limit") and (self.CurrentCommand == "rotate_power_degrees_IMU") and (self.config['imu'] < SensorStatus.DISABLED):
            # Get total degrees rotated
            lastrun = time.time()
            gyrospeed = self.get_gyro_sensor_IMU()[2] #rotate around z-axis
            # Get distance of object infront of robot
            ultrareading = self.get_ultra_sensor()
            totaldegreesrotated += (time.time() - lastrun)*gyrospeed
            # Store as polar coordinates
            scanpoints.append((ultrareading, totaldegreesrotated))
            #self.log((ultrareading, totaldegreesrotated))
        self.stop_all()

        # Convert polar coodinates to cartesian
        scanpointscartesian = []
        for point in scanpoints:
            if point[0] < 255:
                pointcartesian = (robotposition[0]*squaresize+(point[0]*math.cos(math.radians(point[1]))),robotposition[1]*squaresize+(point[0]*math.sin(math.radians(point[1]))))
                self.log("Polar point: " + str(point) + "\n  Cartesian Point: " + str(pointcartesian))
                scanpointscartesian.append(pointcartesian)

        #data['action'] = self.CurrentCommand
        data['elapsed'] = time.time() - starttime
        data['rotated'] = totaldegreesrotated
        data['points'] = scanpointscartesian
        return data
        
    def interpret(self, data, boundaries, squaresize=30):
        self.log("BEGIN INTERPERET")
        points = data['points']
        changedwalls = []
        for point in points:
            # Determine location of walls
            # Get quadrant
            pointrel = [i/squaresize for i in point]
            self.log("point is at: "+str(pointrel))
            quadrant = str(round(pointrel[0]))+","+str(round(pointrel[1]))
            quadrantwalls = boundaries[quadrant]
            dist = math.inf
            wall = None
            # determine closest wall to point
            self.log("Getting closest wall distance")
            for cwall in quadrantwalls:
                cdist = grid.dist(pointrel, cwall.position)
                self.log("distance to wall at " + str(cwall.position)+ " is "+str(cdist))
                if cdist < dist:
                    dist = cdist
                    wall = cwall
            wall.add_points(1)
            self.log("wall at "+str(wall.position)+" has "+str(wall.points)+" points")
            if wall not in changedwalls:
                changedwalls.append(wall)
            #self.log(str(dist) + ": " + str(wall.points))
            pointwall = wall

            # Determine open paths
            # Get quadrant as list
            quadranta = quadrant.split(",")
            quadranta[0]=int(quadranta[0])
            quadranta[1]=int(quadranta[1])
            # Get robot quadrant
            quadrantrobot = (int(round(self.position[0])),int(round(self.position[1])))
            self.log("robot is in " + str(quadrantrobot) + " point is in " + str(quadranta))
            # Get the x and y boundary points (i.e., points ending in .5)
            xpoints = []
            ypoints = []

            if quadranta[0] != quadrantrobot[0]:
                if quadranta[0] > quadrantrobot[0]:
                    for x in range(quadrantrobot[0],quadranta[0]):
                        xpoints.append(x)
                else:
                    for x in range(quadrantrobot[0],quadranta[0],-1):
                        xpoints.append(x)
            if quadranta[1] != quadrantrobot[1]:
                if quadranta[1] > quadrantrobot[1]:
                    for y in range(quadrantrobot[1],quadranta[1]):
                        ypoints.append(y)
                else:
                    for y in range(quadrantrobot[1],quadranta[1],-1):
                        ypoints.append(y)

            self.log(xpoints)
            self.log(ypoints)
            # Get walls and add points
            if xpoints:
                for xpoint in xpoints:
                    xq = xpoint
                    if self.position[0] < point[0]:
                        yq = round(linear_equation_y(self.position,point,xpoint+0.5))
                        wall = boundaries[str(xq)+","+str(yq)][1]
                    else:
                        yq = round(linear_equation_y(self.position,point,xpoint-0.5))
                        wall = boundaries[str(xq)+","+str(yq)][0]
                    self.log("xpoint "+str(xpoint)+": "+str(wall.position)+", "+str(wall.points))
                    self.log("Point Wall at: "+str(pointwall.position))
                    if pointwall != wall:
                        wall.add_points(-1)
                        if wall not in changedwalls:
                            changedwalls.append(wall)
                        self.log("xpoint "+str(xpoint)+": "+str(wall.position)+", "+str(wall.points))
            if ypoints:
                for ypoint in ypoints:
                    yq = ypoint
                    if self.position[1] < point[1]:
                        xq = round(linear_equation_x(self.position,point,ypoint+0.5))
                        wall = boundaries[str(xq)+","+str(yq)][2]
                    else:
                        xq = round(linear_equation_x(self.position,point,ypoint-0.5))
                        wall = boundaries[str(xq)+","+str(yq)][3]
                    self.log("ypoint "+str(ypoint)+": "+str(wall.position)+", "+str(wall.points))
                    self.log(str(pointwall.position))
                    if pointwall != wall:
                        wall.add_points(-1)
                        if wall not in changedwalls:
                            changedwalls.append(wall)
                        self.log("ypoint "+str(ypoint)+": "+str(wall.position)+", "+str(wall.points))
            self.log("\n")
        
        for cwall in changedwalls:
            status = cwall.update_status()
            if status != None:
                for i in cwall.get_quadrants():
                    if i in self.FoundWalls:
                        if cwall not in self.FoundWalls[i]:
                            self.FoundWalls[i].append(cwall)
                    else:
                        self.FoundWalls[i] = [cwall]
            self.log(str(cwall.position) + " is " + str(status) + " and has " + str(cwall.points))
        self.log("found walls: " + str(self.FoundWalls))
        self.removecompletedquads()
        for i in self.FoundWalls:
            self.log("Current found walls are at " + i + " with positions " + str([j.position for j in self.FoundWalls[i]]))
    
    def computedistances(self,quadrants,finalquadrants):
        quadstosearch = [str((int(round(self.position[0])),int(round(self.position[1]))))]
        searchedquads = []
        finaldistances = {}

        layers = 0

        while len(quadstosearch) > 0:
            for quad in quadstosearch:
                if quad in finalquadrants:
                    finaldistances[quad] = layers
                for wall in quadrants[quad]:
                    if wall.open:
                        for i in quadrants:
                            if wall in i and i != quad and i not in searchedquads and i not in quadstosearch:
                                quadstosearch.append(i)
                if quad not in searchedquads:
                    searchedquads.append(quad)
                quadstosearch.remove(quad)
            layers += 1
        
        return finaldistances

    def getinstructions(self,quads,quad):
        quadstosearch = [str((int(round(self.position[0])),int(round(self.position[1]))))]
        searchedquads = []
        pathlink = {}
        finalpath = {}

        while len(quadstosearch) > 0 or finalpath == {}:
            for i in quadstosearch:
                if i == quad:
                    pass
                for wall in quads[i]:
                    if wall.open:
                        for j in quads:
                            if wall in j and j != i and j not in searchedquads and j not in quadstosearch:
                                quadstosearch.append(j)

    def removecompletedquads(self):
        for i in self.FoundWalls:
            if len(self.FoundWalls[i]) == 4:
                self.FoundWalls.remove(i)

    def act(self,data):
        quaddistances = self.computedistances(data,self.FoundWalls)
        cquad = None
        distance = math.inf
        for quad in quaddistances:
            if quaddistances[quad] < distance:
                cquad = quad
                distance = quaddistances[quad]

def linear_equation_y(point1,point2,x):
    m = (point2[1]-point1[1])/(point2[0]-point1[0])
    c = point2[1]-m*point2[0]
    return m*x+c

def linear_equation_x(point1,point2,y):
    if point2[0]==point1[0]:
        return point2[0]
    m = (point2[1]-point1[1])/(point2[0]-point1[0]) 
    c = point2[1]-m*point2[0]
    return (y-c)/m

def decimal_range(start, stop, step=1):
    if start < stop:
        while start < stop:
            yield start
            start += step
    elif start > stop:
        while start > stop:
            yield start
            start -= step
    else:
        yield start

# Only execute if this is the main file, good for testing code
if __name__ == '__main__':
    logging.basicConfig(filename='logs/robot.log', level=logging.INFO)
    boundaries = grid.generate_map()
    ROBOT = Robot(timelimit=10)  #10 second timelimit before
    bp = ROBOT.BP
    ROBOT.configure_sensors() #This takes 4 seconds
    try:
        """ROBOT.log(ROBOT.calibrate_position(42))
        ROBOT.log(ROBOT.get_orientation_IMU())
        # Scan school test
        points = ROBOT.scan(9, -360, 42, 30)
        # Scan Comp test
        #points = ROBOT.scan(9, -360, 15)
        ROBOT.log(points)
        
        ROBOT.interpret(points, boundaries)"""
        distcm = ROBOT.move_distance_encoder(30)
        print(distcm)
        sensordict = ROBOT.get_all_sensors()
        ROBOT.log(sensordict)
    finally:
        ROBOT.safe_exit()

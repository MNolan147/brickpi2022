#This is where your main robot code resides. It extendeds from the BrickPi Interface File
#It includes all the code inside brickpiinterface. The CurrentCommand and CurrentRoutine are important because they can keep track of robot functions and commands. Remember Flask is using Threading (e.g. more than once process which can confuse the robot)
from interfaces.brickpiinterface import *
import grid
import global_vars as GLOBALS
import logging
import math

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
        return
        
    def scan(self, power, degrees, marginoferror=3, robotposition=(0,0)):
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
        
        starttime = time.time(); timelimit = starttime + self.timelimit
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
            pointcartesian = (robotposition[0]+(point[0]*math.cos(point[1])),robotposition[1]+(point[0]*math.sin(point[1])))
            scanpointscartesian.append(pointcartesian)

        #data['action'] = self.CurrentCommand
        data['elapsed'] = time.time() - starttime
        data['rotated'] = totaldegreesrotated
        data['points'] = scanpointscartesian
        return data
        
    def interpret(self, data, boundaries):
        points = data['points']
        changedwalls = []
        for point in points:
            # Determine location of walls
            # Get quadrant
            quadrant = str(round(point[0]/30))+","+str(round(point[1]/30))
            quadrantwalls = boundaries[quadrant]
            dist = math.inf
            wall = None
            # determine closest wall to point
            for cwall in quadrantwalls:
                cdist = grid.dist(point, cwall.position)
                if cdist < dist:
                    dist = cdist
                    wall = cwall
            wall.add_points(1)
            if wall not in changedwalls:
                changedwalls.append(wall)
            #self.log(str(dist) + ": " + str(wall.points))

            # Determine open paths
            quadranta = quadrant.split(",")
            quadranta[0]=int(quadranta[0])
            quadranta[1]=int(quadranta[1])
            quadrantrobot = (int(round(self.position[0]/30)),int(round(self.position[1]/30)))
            self.log("robot at " + str(quadrantrobot) + " point at " + str(quadranta))
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

            for point in xpoints:
                yq = round(linear_equation_y(quadrantrobot,quadranta,point))
                xq = math.floor(point)
                wall = boundaries[str(xq)+","+str(yq)][0]
                wall.add_points(-1)
                if wall not in changedwalls:
                    changedwalls.append(wall)
            for point in ypoints:
                xq = round(linear_equation_x(quadrantrobot,quadranta,point))
                yq = math.floor(point)
                wall = boundaries[str(xq)+","+str(yq)][2]
                wall.add_points(-1)
                if wall not in changedwalls:
                    changedwalls.append(wall)
        
        for wall in changedwalls:
            wall.update_status()




    # Create a function to move time and power which will stop if colour is detected or wall has been found
    
    
    

    #Create a function to search for victim
    

    
    
    
    #Create a routine that will effective search the maze and keep track of where the robot has been.

def linear_equation_y(point1,point2,x):
    m = (point2[1]-point1[1])/(point2[0]-point1[0])
    c = point2[1]-m*point2[0]
    return m*x+c

def linear_equation_x(point1,point2,y):
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
    time.sleep(2)
    try:
        points = ROBOT.scan(12, 360, 12)
        ROBOT.log(points)
        
        ROBOT.interpret(points, boundaries)

        sensordict = ROBOT.get_all_sensors()
        ROBOT.log(sensordict)
    finally:
        ROBOT.safe_exit()

import matplotlib.pyplot as plt
import numpy as np
import math
import json
import traceback
import sys

# CONTROL PARAMETERS
BOX_SIZE = 0.1
ROOM_SIZE = 15
PROB_HIT = 0.9
PROB_MISS = 0.4
ROBOT_WIDTH = 0.2


class Cols:
    HEADER = '\033[34m'
    INFO = '\033[92m'
    FAILED = '\033[91m'
    ENDC = '\033[0m'
    
def print_header(message):
    print(f"{Cols.HEADER}"+message+f"{Cols.ENDC}")

def print_info(message):
    print(f"{Cols.INFO}"+message+f"{Cols.ENDC}")

def print_failed(message):
    print(f"{Cols.FAILED}"+message+f"{Cols.ENDC}")



class Converter:
    def __init__(self):
        self.pi = 3.141592653589793

    def polar2cart(self, rho, phi):
        x = rho * math.cos(phi)
        y = rho * math.sin(phi)
        return [x, y]
    

class Robot:
    def __init__(self):
        self.laserRange = 10
        self.laserShift = 0.1
        self.width = ROBOT_WIDTH

    def transform_kinematic(self, globalPos, angle, dist):
        newPos = np.array([1,1,0], dtype=np.float)
        newPos[0] = dist * math.cos(globalPos[2] + angle) + globalPos[0]
        newPos[1] = dist * math.sin(globalPos[2] + angle) + globalPos[1]
        newPos[2] = globalPos[2]
        return newPos


class OccupancyGridMap(Robot):
    def __init__(self):
        super().__init__()
        self.boxSize = BOX_SIZE
        self.roomSize = ROOM_SIZE
        self.xSize = int(self.roomSize / self.boxSize)
        self.ySize = int(self.roomSize / self.boxSize)      
        self.probHit = PROB_HIT
        self.probMiss = PROB_MISS
        self.thHit = 1
        self.thMiss = -1
        self.fieldMap = np.zeros((self.xSize, self.ySize))
        self.robotPathX = []
        self.robotPathY = []
        self.fig = None
        self.mapFig = None

    def return_field_map(self):
        return self.fieldMap

    def add_point_to_path(self, x, y):
        self.robotPathX.append(x)
        self.robotPathY.append(y)
    
    def return_robot_path(self):
        return [self.robotPathX, self.robotPathY]

    def glob2coord(self, x, y):
        coords = np.array([0,0], dtype=np.int)
        coords[0] = int( (x/self.boxSize) + self.xSize/2 )
        coords[1] = int( (y/self.boxSize) + self.ySize/2 )
        return coords

    def global_pos_of_box(self, box):
        pos = np.array([0,0], dtype=np.float)
        pos[0] = (box[0] - self.xSize/2) * self.boxSize
        pos[1] = (box[1] - self.ySize/2) * self.boxSize
        return pos
    
    def hit(self, field):
        field = field + np.log(self.probHit/(1-self.probHit))
        if field > self.thHit:
            field = self.thHit
        return field

    def miss(self, field):
        field = field + np.log(self.probMiss/(1-self.probMiss))
        if field < self.thMiss:
            field = self.thMiss
        return field

    def dist_point_to_line(self, A, B, C, xPoint, yPoint):
        return np.absolute(A * xPoint + B * yPoint + C) / np.sqrt(A*A + B*B)

    def get_missed_fields(self, startPoint, endPoint):
        startPoint = self.glob2coord(startPoint[0], startPoint[1])
        endPoint = self.glob2coord(endPoint[0], endPoint[1])
        x1 = startPoint[0]
        y1 = startPoint[1]
        x2 = endPoint[0]
        y2 = endPoint[1]

        dx = x2 - x1
        dy = y2 - y1

        # Determine how steep the line is
        is_steep = abs(dy) > abs(dx)
    
        # Rotate line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2
    
        # Swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
    
        # Recalculate differentials
        dx = x2 - x1
        dy = y2 - y1
    
        # Calculate error
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1
    
        # Iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
    
        # Reverse the list if the coordinates were swapped
        if swapped:
            points.reverse()
        return points


    def update_map(self, pose, scan):
        robotPose = np.array([0,0,0], dtype=np.float)
        for i in range(3):
            robotPose[i] = pose[i]
        robotPose[2] = robotPose[2] / 180 * np.pi
        sensorPose = Robot.transform_kinematic(robotPose, 0, Robot.laserShift)

        x = np.arange(0, len(scan))
        angles = (np.pi/len(scan))*x - np.pi/2

        for i in x:
            if np.isinf(scan[i]) or np.isnan(scan[i]):
                continue
            else:
                targetPoint = Robot.transform_kinematic(sensorPose, angles[i], scan[i])
                targetCoord = self.glob2coord(targetPoint[0], targetPoint[1])
                x = int(targetCoord[0])
                y = int(targetCoord[1])

                if x>=int(self.roomSize / self.boxSize) or y>=int(self.roomSize / self.boxSize):
                    print_failed("Error: Room size is too small to plot the element x: {} y: {}. \n Please, increate the room space...".format(x,y))
                else:
                    # print("x: {}  y: {}".format(x, y))
                    self.fieldMap[y][x] = self.hit(self.fieldMap[y][x])
                    missed1 = self.get_missed_fields(sensorPose, targetPoint)                
                    # missed2 = self.getMissedFields_2(sensorPose, targetPoint)

                    for box in missed1:
                        self.fieldMap[box[1]][box[0]] = self.miss(self.fieldMap[box[1]][box[0]])

        return self.fieldMap

    def compute_prob_map(self):
        return 1 - (1/(1 + np.exp(self.fieldMap)))	

    def initial_plot(self):
        self.fig = plt.figure()
        self.fieldMap[0][0] = 0.01
        # self.mapFig = plt.imshow(self.computeProbMap(), interpolation="nearest", cmap='Blues', vmin=0, vmax=1, origin='lower')
        self.mapFig = plt.imshow(self.compute_prob_map(), interpolation="nearest", cmap='Blues', origin='lower')
        plt.colorbar()

    def update_plot(self):
        self.mapFig.set_data(self.compute_prob_map())
        plt.plot(self.return_robot_path()[0], self.return_robot_path()[1], 'r-', linewidth=1)
        plt.show(block=False)
        plt.pause(0.1)
    
    def show_plot(self):
        plt.show()


class PathFinder(Robot):
    def __init__(self):
        self.ogmap = None
        self.binmap = None
        self.wfmap = None
        self.path = None
        self.th = 0.5
        self.robot_width = Robot.width
        self.bin_xSize = None
        self.bin_ySize = None
        self.bin_size = int(self.robot_width/BOX_SIZE)

    def initialize(self, in_ogmap):
        self.ogmap = in_ogmap
        self.bin_xSize = int(ROOM_SIZE/self.robot_width)
        self.bin_ySize = int(ROOM_SIZE/self.robot_width)
        self.binmap = np.zeros((self.bin_xSize, self.bin_ySize))
        self.wfmap = np.zeros((self.bin_xSize, self.bin_ySize))
    
    def get_wavefront_map(self):
        return self.wfmap
        
    def plot_binary_map(self):
        self.fig = plt.figure()
        # self.mapFig = plt.imshow(self.computeProbMap(), interpolation="nearest", cmap='Blues', vmin=0, vmax=1, origin='lower')
        self.mapFig = plt.imshow(self.binmap, interpolation="nearest", origin='lower')
        plt.colorbar()
    
    def show_binary_map(self):
        plt.show()
        plt.pause(0.1)
    
    def binearize_occupany_grid_map(self):
        try:
            for x in range(0, self.bin_xSize):
                for y in range(0, self.bin_ySize):
                    
                    box_list = []
                    for i in range(0, self.bin_size):
                        for j in range(0, self.bin_size):
                            # print((x * self.bin_size) + i, "  ", (y * self.bin_size) + j)
                            box_list.append(self.ogmap[(x * self.bin_size) + i][(y * self.bin_size) + j])
                    if max(box_list) >= self.th: # jest przeszkoda 
                        self.binmap[x][y] = 1
            print_info("OGM binearized")
        except Exception as ex:
            print_failed("OGM BIN ERROR: " + str(ex))
            traceback.print_exc(file=sys.stdout)

    def prepare_wavefront_map(self, start):
        self.wfmap.fill(np.inf)
        for x in range(0, self.bin_xSize):
                for y in range(0, self.bin_ySize):
                    if self.binmap[x][y] == 1:
                        self.wfmap[x][y] = -1
                    if (x == start[0]) and (y == start[1]):
                        self.wfmap[x][y] = 0
        

    def check_neighbour(self, pos, dir):
        #check if the neighbour is not an obstacle
        if (pos[1]+1 < self.bin_ySize) and (pos[0]+1 < self.bin_xSize) and (pos[1]-1 >= 0) and (pos[0]-1 >= 0):
            if dir == 0:
                if (self.wfmap[pos[0]][pos[1]+1] == -1) or (self.wfmap[pos[0]][pos[1]+1] != np.inf):
                    return False
            elif dir == 1:
                if (self.wfmap[pos[0]+1][pos[1]] == -1) or (self.wfmap[pos[0]+1][pos[1]] != np.inf):
                    return False
            elif dir == 2:
                if (self.wfmap[pos[0]][pos[1]-1] == -1) or (self.wfmap[pos[0]][pos[1]-1] != np.inf):
                    return False
            elif dir == 3:
                if (self.wfmap[pos[0]-1][pos[1]] == -1) or (self.wfmap[pos[0]-1][pos[1]] != np.inf):
                    return False
            return True
        else:
            return False


    def create_wavefront_map(self, start, stop):
        self.prepare_wavefront_map(start)
        in_target = False
        iter = 1
        start_found = False
        while not in_target:
            for x in range(0, self.bin_xSize):
                for y in range(0, self.bin_ySize):
                    # look for tiles with current step number and give them value of step+1
                    if (self.wfmap[x][y] == iter-1):
                        for i in range(4):
                            if self.check_neighbour((x, y), i):
                                if i == 0:
                                    self.wfmap[x][y+1] = iter
                                elif i == 1:
                                    self.wfmap[x+1][y] = iter
                                elif i == 2:
                                    self.wfmap[x][y-1] = iter
                                elif i == 3:
                                    self.wfmap[x-1][y] = iter
                
                    elif (self.wfmap[stop[0]][stop[1]] > -1) and (self.wfmap[stop[0]][stop[1]] != np.inf):
                        in_target = True
                        break      
            iter += 1


    def find_path(self, start, stop):
        path = []
        currPoint = [start[0], start[1]]
        step = self.wfmap[start[0]][start[1]] +1

        while(1):
            path.append([currPoint[0], currPoint[1]])
            step = step - 1
            
            if (currPoint[0] == stop[0] and currPoint[1] == stop[1]):
                break
            if (currPoint[0] > 0):
                if (self.wfmap[currPoint[0]][currPoint[1] - 1] == step - 1):
                    currPoint[1] -= 1
                    continue
            if (currPoint[1] > 0):
                if (self.wfmap[currPoint[0]-1][currPoint[1]] == step - 1):
                    currPoint[0] -= 1
                    continue
            if (currPoint[0] < self.bin_xSize - 1):
                if (self.wfmap[currPoint[0]][currPoint[1] + 1] == step - 1):
                    currPoint[1] += 1
                    continue
            if (currPoint[1] < self.bin_ySize - 1):
                if (self.wfmap[currPoint[0] + 1][currPoint[1]] == step - 1):
                    currPoint[0] += 1
                    continue
        return path

    
if __name__ == '__main__':
    try:
        _filepath = 'data/ex2.json'
        json_file = open(_filepath)
        data = json.load(json_file) 
        print_info(str(_filepath) + " file loaded...")
        print_info("Data lenght: " + str(len(data)))
        FILE_LOADED = True
    except Exception as ex:
        print_failed(str(_filepath) + " file loading FAILED")
        FILE_LOADED = False

    if FILE_LOADED:
        OGM = OccupancyGridMap()
        
        Robot = Robot()
        Conv = Converter()
        PF = PathFinder()

        for i in range(0, len(data)):
            print_info("  Iteration: " + str(i))
            scan = data[i]['scan']
            pose = data[i]['pose']

            robotPose = OGM.glob2coord(pose[0], pose[1])
            OGM.add_point_to_path(robotPose[0], robotPose[1])
            fm = OGM.update_map(pose, scan)
            if i == 0:
                OGM.initial_plot()
            else:
                OGM.update_plot()
        

        PF.initialize(in_ogmap=OGM.return_field_map())
        PF.binearize_occupany_grid_map()
        PF.plot_binary_map()

        # initial y,x
        robot = (6, 54)
        target = (65, 30)
        
        plt.plot([robot[1], target[1]], [robot[0], target[0]], 'b-.')
        PF.create_wavefront_map(robot, target)
        print_info("WFmap created")
        path = PF.find_path(target, robot)
        print_info("Found path: " + str(path))

        fig = plt.figure()
        plt.imshow(PF.get_wavefront_map(), interpolation="nearest", origin='lower')
        ys, xs = map(list, zip(*path))
        plt.plot(xs, ys, 'r-', linewidth=2)

        plt.colorbar()
        plt.show()
    else:
        print_failed("Could not start algorithm")

    print_header("### PROGRAM FINISHED ###")

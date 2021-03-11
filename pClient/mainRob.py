
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14
CELL_SIDE=2

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def set_start_cell(self, start_cell):
        self.start_cell = start_cell

    def set_targets(self, targets):
        self.targets = targets

    def set_challenge(self, challenge):
        self.challenge = challenge

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'
        self.coords_offset = None
        self.dest_cell = self.start_cell
        self.dest_angle = 0

        # get directions list (hardcoded for now)
        self.path = [(2,3), (3,3), (3,4), (4,4), (4,5), (5,5), (6,5), (6,6), (7,6), (8,6), (9,6), (10,6), (11,6), (11,5), (11,4), (11,3), (12,3)]
        self.path_index = 0
        self.get_next_dest()

        # number of targets visited
        self.num_targets_visited = 0
        on_target = False

        while True:
            self.readSensors()

            if not self.coords_offset:
                #calcular offset
                self.coords_offset = (self.measures.x - self.transform_to_gps_pos(self.start_cell[0]), self.measures.y - self.transform_to_gps_pos(self.start_cell[1]))

            if self.measures.endLed:
                print(self.robName + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                    self.num_targets_visited += 1
                else:
                    self.decide()
                if self.measures.ground >= 0 and not on_target:
                    self.setVisitingLed(True)
                    on_target = True
                elif self.measures.ground < 0 and on_target:
                    on_target = False
                
            elif state=='wait':
                # return to initial position only if all targets were visited
                if self.num_targets_visited == len(self.targets):
                    self.setReturningLed(True)
                else:
                    state='run'
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                    self.num_targets_visited = 0
                    on_target = False
                    self.path_index = 0
                    self.path.pop()
                    self.path.reverse()
                    self.get_next_dest()

            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                if self.measures.ground>=0 and not on_target:
                    self.setVisitingLed(True)
                    self.num_targets_visited += 1
                    on_target = True
                elif self.measures.ground < 0 and on_target:
                    on_target = False
                if self.start_cell[0]==self.cur_cell[0] and self.start_cell[1]==self.cur_cell[1]:
                    self.finish()
                else:
                    self.decide()
            

    def wander(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        if    self.measures.irSensor[center_id] > 5.0\
           or self.measures.irSensor[left_id]   > 5.0\
           or self.measures.irSensor[right_id]  > 5.0\
           or self.measures.irSensor[back_id]   > 5.0:
            print('Rotate left')
            self.driveMotors(-0.1,+0.1)
        elif self.measures.irSensor[left_id]> 2.7:
            print('Rotate slowly right')
            self.driveMotors(0.1,0.0)
        elif self.measures.irSensor[right_id]> 2.7:
            print('Rotate slowly left')
            self.driveMotors(0.0,0.1)
        else:
            print('Go')
            self.driveMotors(0.1,0.1)

    # --------------------------------------------

    def decide(self):
        ROT_THRESHOLD = 0
        MOVE_THRESHOLD = 0.2

        angle_dif = self.dest_angle - self.measures.compass
        
        if abs(angle_dif) == 360:
            angle_dif = 0
        elif abs(angle_dif) > 180:
            angle_dif *= -1
        if angle_dif > ROT_THRESHOLD:
            self.driveMotors(-0.02, 0.02)
        elif angle_dif < ROT_THRESHOLD:
            self.driveMotors(0.02, -0.02)
        else:
            # angled with destination
            # convert next cell coordinates to gps coords
            next_pos = (self.transform_to_gps_pos(self.dest_cell[0])+self.coords_offset[0], self.transform_to_gps_pos(self.dest_cell[1])+self.coords_offset[1])
            if abs(self.measures.x - next_pos[0]) > MOVE_THRESHOLD or abs(self.measures.y - next_pos[1]) > MOVE_THRESHOLD:
                self.driveMotors(0.1,0.1)
            else:
                self.get_next_dest()


    def get_next_dest(self):

        self.cur_cell = self.dest_cell
        if self.path_index < len(self.path):
            self.dest_cell = self.path[self.path_index]
            self.path_index += 1

        if self.dest_cell[0] - self.cur_cell[0] > 0:
            self.dest_angle = 0
        elif self.dest_cell[0] - self.cur_cell[0] < 0:
            self.dest_angle = 180
        elif self.dest_cell[1] - self.cur_cell[1] > 0:
            self.dest_angle = 90
        elif self.dest_cell[1] - self.cur_cell[1] < 0:
            self.dest_angle = -90


    def transform_to_gps_pos(self, axis):
        return axis*CELL_SIDE + CELL_SIDE/2

    # --------------------------------------------

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)/3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c/3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None
start_cell = None
targets =[]
challenge = 1

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    elif (sys.argv[i] == "--start" or sys.argv[i] == "-s") and i != len(sys.argv) - 1:
        start_cell_row, start_cell_col = sys.argv[i + 1].split(',')
        start_cell = (int(start_cell_row), int(start_cell_col))
    elif (sys.argv[i] == "--targets" or sys.argv[i] == "-t") and i != len(sys.argv) - 1:
        trgts = sys.argv[i + 1].split('_')
        for t in trgts:
            target_row, target_col = t.split(',')
            targets.append((int(target_row), int(target_col)))   
        #target_row, target_col = sys.argv[i + 1].split(',')
        #targets.append((int(target_row), int(target_col)))
    elif (sys.argv[i] == "--challenge" or sys.argv[i] == "-c") and i != len(sys.argv) - 1:
        challenge = int(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    if start_cell != None:
        rob.set_start_cell(start_cell)
    if targets:
        rob.set_targets(targets)
    rob.set_challenge(challenge)
    
    rob.run()

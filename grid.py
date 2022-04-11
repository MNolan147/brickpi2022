import math, json

class Boundary():

    def __init__(self, position):
        self.position = position
        if math.floor(position[0]) == position[0]:
            self.orientaion = 0
        else:
            self.orientaion = 1
        self.points = 0
        self.open = None
    
    def add_points(self, points):
        self.points += points
        return self.points
    
    def update_status(self):
        if self.points > 5:
            self.open = False
        elif self.points < -5:
            self.open = True
        return self.open

def generate_map(size=25):
    boundaries = {}

    for i in range(-size, size+1):
        xcm = i*30
        for j in range(-size, size+1):
            ycm = j*30
            positions = [(xcm-0.5,ycm), (xcm+0.5,ycm), (xcm,ycm+0.5), (xcm,ycm-0.5)]
            bpositions = []
            for k in positions:
                bpositions.append(Boundary(k))
            boundaries[str(i)+","+str(j)] = bpositions
    
    return boundaries

def dist(p, q):
    return math.sqrt((p[0]-q[0])**2+(p[1]-q[1])**2)

if __name__ == '__main__':
    print(generate_map())
    

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
        if self.points > 2:
            self.open = False
        elif self.points < -2:
            self.open = True
        return self.open
    
    def get_quadrants(self):
        if self.position[1] == math.floor(self.position[1]):
            return [str(int(self.position[0]+0.5))+","+str(self.position[1]),str(int(self.position[0]-0.5))+","+str(self.position[1])]
        elif self.position[0] == math.floor(self.position[0]):
            return [str(self.position[0])+","+str(int(self.position[1]+0.5)),str(self.position[0])+","+str(int(self.position[1]-0.5))]


def generate_map(size=25):
    boundaries = {}
    positionsd = {}
    for i in range(-size, size+1):
        for j in range(-size, size+1):
            positions = [(i-0.5,j), (i+0.5,j), (i,j+0.5), (i,j-0.5)]
            bpositions = []
            for k in positions:
                if str(k[0])+','+str(k[1]) in positionsd:
                    bpositions.append(positionsd[str(k[0])+','+str(k[1])])
                else:
                    bpositions.append(Boundary(k))
                    positionsd[str(k[0])+','+str(k[1])] = bpositions[-1] 
            boundaries[str(i)+','+str(j)] = bpositions
    #print(positionsd)
    return boundaries

def dist(p, q):
    return math.sqrt((p[0]-q[0])**2+(p[1]-q[1])**2)

if __name__ == "__main__":
    map = generate_map()
    for i in map:
        print("coords "+i+" walls "+str([j.position for j in map[i]]))
    

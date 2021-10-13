import math
import random
import matplotlib.pyplot as plt
import time

traject = []
trajectp = []
last_particle = 0

landmarks = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
world_size = 100.0
p = []
N = 1000



class robot:
    def __init__(self, initial):
        if(initial):
            self.x = 30  # initialise with random
            self.y = 30
            self.orientation = 0.0
        else:
            self.x = random.random() * world_size  # initialise with random
            self.y = random.random() * world_size
            self.orientation = random.random() * 2.0 * math.pi
       # draw_particles(self.x,self.y,self.orientation)
        self.forward_noise = 0.0
        self.turn_noise = 0.0
        self.sense_noise = 0.0

    def set(self, new_x, new_y, new_orientation):
        if new_x < 0 or new_x >= world_size:
            raise ValueError('X coordinate out of bound')
        if new_y < 0 or new_y >= world_size:
            raise ValueError('Y coordinate out of bound')
        if new_orientation < 0 or new_orientation >= 2 * math.pi:
            raise ValueError('Orientation must be in [0..2pi]')
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)

    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.forward_noise = float(new_f_noise)
        self.turn_noise = float(new_t_noise)
        self.sense_noise = float(new_s_noise)

    def sense(self):
        Z = []
        for i in range(len(landmarks)):
            dist = math.sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            Z.append(dist)
        return Z

    def move(self, turn, forward):
        if forward < 0:
            raise ValueError('Robot cant move backwards')


        # turn, and add randomness to the turning command
        orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
        orientation %= 2 * math.pi

        # move, and add randomness to the motion command
        dist = float(forward) + random.gauss(0.0, self.forward_noise)
        x = self.x + (math.cos(orientation) * dist)
        y = self.y + (math.sin(orientation) * dist)
        x %= world_size  # cyclic truncate
        y %= world_size

        # set particle
        res = robot(0)
        res.set(x, y, orientation)
        res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
        return res

    def Gaussian(self, mu, sigma, x):
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return math.exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / math.sqrt(2.0 * math.pi * (sigma ** 2))

    def measurement_prob(self, measurement):
        # calculates how likely a measurement should be
        prob = 1.0
        for i in range(len(landmarks)):
            dist = math.sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            prob *= self.Gaussian(dist, self.sense_noise, measurement[i])
        return prob


def move(myrobot,step,turn,forward):
    global p
    for rd in range(step):
        myrobot = myrobot.move((math.pi / 180) * turn, forward)
        traject.append(myrobot)
        Z = myrobot.sense()
        p2 = []
        for i in range(N):
            p2.append(p[i].move((math.pi / 180) * turn, forward))
        p = p2
        draw_particles(p, myrobot,2,0)
        # given the particle's location, how likely measure it as Z
        w = []

        for i in range(N):
            w.append(p[i].measurement_prob(Z))
        # for rob in p:
        #     prob = rob.measurement_prob(Z)  # Z remains the same
        #     w.append(prob)
        # resampling particles based on prabability weights
        p3 = []
        index = int(random.random() * N)
        beta = 0
        mw = max(w)

        for i in range(N):
            beta += random.random() * 2 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % N
            p3.append(p[index])
        p = p3
        trajectp.append(p[w.index(min(w))])
        draw_particles(p, myrobot,1,p[w.index(min(w))])
    return myrobot
def draw_particles(particles,robotpose,drawp,particle_pose):
    global last_particle
    plt.clf()
    lx = []
    ly = []
    x3 = []
    y3 = []
    x4 = []
    y4 = []
    # Set chart title.
    plt.title("Robot World ")
    # Set x, y label text.
    plt.xlim(0, world_size)
    plt.ylim(0, world_size)
    plt.xlabel("X")
    plt.ylabel("Y")
    for i in traject:
        x3.append(i.x)
        y3.append(i.y)
    for i in trajectp:
        x4.append(i.x)
        y4.append(i.y)
    #Draw the robot
    if(drawp==1):
        plt.scatter(particle_pose.x, particle_pose.y, s=200, color='blue')
        last_particle=particle_pose
        x2 = math.cos(particle_pose.orientation) * 1 + particle_pose.x
        y2 = math.sin(particle_pose.orientation) * 1 + particle_pose.y
        lx.append([particle_pose.x, x2])
        ly.append([particle_pose.y, y2])
    elif(drawp==2):
        plt.scatter(last_particle.x, last_particle.y, s=200, color='blue')
        x2 = math.cos(last_particle.orientation) * 1 + last_particle.x
        y2 = math.sin(last_particle.orientation) * 1 + last_particle.y
        lx.append([last_particle.x, x2])
        ly.append([last_particle.y, y2])

    plt.scatter(robotpose.x, robotpose.y, s=200,color='green')
    x2 = math.cos(robotpose.orientation) * 1 + robotpose.x
    y2 = math.sin(robotpose.orientation) * 1 + robotpose.y
    lx.append([robotpose.x, x2])
    ly.append([robotpose.y, y2])

    plt.plot(x3, y3, color='green')
    plt.plot(x4, y4, color='blue')
    # Draw the significant particles
    for i in landmarks:
        plt.scatter(i[0], i[1], s=1000, color='blue')
    for i in particles:
        plt.scatter(i.x, i.y, s=10,color='blue')
        x2 = math.cos(i.orientation) * 1 + i.x
        y2 = math.sin(i.orientation) * 1 + i.y
        lx.append([i.x, x2])
        ly.append([i.y, y2])
    for i in range(len(lx)):
        plt.plot(lx[i], ly[i], color='red')
    plt.pause(0.000000001)





# Init actual location
myrobot = robot(1)
traject.append(myrobot)
draw_particles(p,myrobot,0,0)
time.sleep(7)
myrobot = myrobot.move(0, 5.0)
traject.append(myrobot)
draw_particles(p,myrobot,0,0)
Z = myrobot.sense()

# initialise randomly guessed particles
for i in range(N):
    x = robot(0)
    x.set_noise(0.05, 0.05, 5.0)
    p.append(x)

wi=[]
for i in range(N):
    wi.append(p[i].measurement_prob(Z))
draw_particles(p,myrobot,1,p[wi.index(min(wi))])
myrobot=move(myrobot,7,0,5)
myrobot=move(myrobot,1,90,5)
myrobot=move(myrobot,7,0,5)
myrobot=move(myrobot,1,90,5)
myrobot=move(myrobot,7,0,5)
myrobot=move(myrobot,1,90,5)
myrobot=move(myrobot,1,-90,5)
myrobot=move(myrobot,3,0,5)
myrobot=move(myrobot,1,-90,5)
myrobot=move(myrobot,4,0,5)
print("done")
plt.show()
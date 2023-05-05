import numpy as np
import pygame
from numpy import dot, array, cos, sin, linspace, radians, arccos, arcsin
from numpy.linalg import norm
from queue import PriorityQueue
import time
from queue import PriorityQueue
import matplotlib.backends.backend_agg as agg
import matplotlib.pyplot as plt

WIDTH = 800
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("HERMES (EMHRT)")

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 255, 0)
YELLOW = (255, 230, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)


class Node:
    def __init__(self, pos, angle, children=None, previous=None):
        if children is None:
            children = []
        self.pos = pos
        self.angle = angle
        self.node_vector = array((cos(self.angle), sin(self.angle)))
        self.error = 0
        self.children = children
        self.previous = previous
        self.generation = 0 if previous is None else self.previous.generation+1
        self.cum_error = 0
        self.closed = False
        self.h = None

    def close(self):
        self.closed = True

    def dist_to_goal(self, goal):
        return norm(array(self.pos)-array(goal.pos))

    def emh(self, u, v, goal):  # calculate heuristic cost value for the Node
        em_vector = array([u, v])
        n_ = array([cos(self.angle), sin(self.angle)])  # normalized direction vector
        n = array(self.pos)
        O = array(goal.pos)
        em_offset_theta = arccos(abs(dot(em_vector, n_)/(norm(em_vector)*norm(n_))))
        d2goal = norm(n-O)
        O_vector = (n-O) # vector pointing to object
        O_offset_theta = arccos(abs(dot(O_vector, n_)/(norm(O_vector)*norm(n_)))) # offset angle between heading vector and objective
        return d2goal*(np.exp(em_offset_theta/np.pi) + 1.1*np.exp(O_offset_theta/np.pi) + 1)

    def calculate_curl(self, charges):  # calculate curl vector at the Node's position
        u = 0
        v = 0
        for charge in charges:
            if charge.pol == 0:
                continue
            u += charge.pol * (charge.pos[1] - self.pos[1]) / ((charge.pos[0] - self.pos[0]) ** 2 + (charge.pos[1] - self.pos[1]) ** 2)
            v += -charge.pol * (charge.pos[0] - self.pos[0]) / ((charge.pos[0] - self.pos[0]) ** 2 + (charge.pos[1] - self.pos[1]) ** 2)
        return array([u, v])

    def div_map(self, charges):  # calculate divergence vector at the Node's position
        u = 0
        v = 0
        for charge in charges:
            exp = 2 #(3 if charge.pol is 0 else 2)
            gain = 0.2 if charge.pol == 0 else 1
            u += -gain*(charge.pos[0] - self.pos[0]) / (abs(charge.pos[0] - self.pos[0]) ** exp + abs(charge.pos[1] - self.pos[0]) ** exp)
            v += -gain*(charge.pos[1] - self.pos[1]) / (abs(charge.pos[0] - self.pos[0]) ** exp + abs(charge.pos[1] - self.pos[1]) ** exp)
        return array([u, v])

    def get_emv(self, charges):  # covers the case of no charges
        if len(charges) < 1:
            return array([0, 0])
        else:
            return self.calculate_curl(charges=charges) + 0.3*self.div_map(charges=charges)

    def update_error(self, charges):  # euclidean heuristic function
        emvector = self.calculate_curl(charges=charges) + 0.3*self.div_map(charges=charges)
        self.error = norm(self.pos-emvector)

    def draw(self, win):  # Draw the node
        pygame.draw.circle(win, PURPLE, self.pos, 2)
        if self.previous:
            pygame.draw.line(win, GREEN, self.pos, self.previous.pos)
        for child in self.children:
            child.draw(win)

    def bottom_view(self):  # return the bottom view of the path planning tree
        bottom_nodes = []
        if not self.children:
            bottom_nodes.append(self)
        else:
            for child in self.children:
                bottom_nodes.append(child.bottom_view())
        return array(bottom_nodes).ravel()

    def path2root(self, path):  # returns a path as a list of Node objects
        if not self.previous:
            path.append(self)
        else:
            path.append(self)
            self.previous.path2root(path)
            # path.append(self.previous.path2root(path))
        return array(path).ravel()

    def reached_goal(self, goal):
        return norm(goal.pos-self.pos) < goal.radius

    def __lt__(self, other):
        return False


class Charge:  # Charge Class
    def __init__(self, pos, pol):
        self.pos = pos
        self.pol = pol
        self.radius = 5

    def draw_charge(self, win):  # Draw the Charges
        color = [RED, YELLOW, GREEN]
        pygame.draw.circle(win, color[self.pol+1], self.pos, self.radius)
        pygame.draw.circle(win, color[self.pol+1], self.pos, self.radius+20, 2)


class Goal:  # Goal class
    def __init__(self, pos, radius):
        self.pos = pos
        self.radius = radius

    def draw_goal(self, win):  # Draw Goal
        pygame.draw.circle(win, ORANGE, self.pos, self.radius)


class Agent:
    def __init__(self, pos, orientation):
        self.pos = pos
        self.angle = orientation


class PathPlanner:  # manages the tree expansion, and checks if the goal has been reached
    def __init__(self, init_node, goal, cone_angle=radians(30), growth_rate=3, dx=50, live=10):
        self.tree = init_node
        self.charges = []
        self.bottom_view = [self.tree]
        self.PriorityQ = PriorityQueue()
        self.PriorityQ.put((self.tree.dist_to_goal(goal), self.tree))
        self.agent = Agent(self.tree.pos, self.tree.angle)
        self.cone_angle = cone_angle
        self.growth_rate = growth_rate
        self.edge_length = dx
        self.live = live
        self.goal = goal
        self.paths = []
        self.boat_width = 20

    def is_collision(self, pos, charges):
        for charge in charges:
            if norm(pos-charge.pos) < charge.radius + self.boat_width:
                return True
        return False

    def grow_tree(self):
        # grow tree based on movement parameters
        node_angles = array(linspace(self.cone_angle/2, -self.cone_angle/2, self.growth_rate))
        _, node = self.PriorityQ.get()
        for angle in node_angles:
            node_pos = node.pos + array([self.edge_length*cos(node.angle+angle), self.edge_length*sin(node.angle+angle)])
            if self.is_collision(node_pos, self.charges):
                continue
            node_angle = angle + node.angle
            new_node = Node(node_pos, node_angle, children=[], previous=node)
            node.children.append(new_node)
            u, v = new_node.get_emv(self.charges)
            h = new_node.emh(u, v, self.goal)
            if new_node.dist_to_goal(self.goal) < self.goal.radius/2:
                path = []
                self.paths.append(node.path2root(path))
                continue
            self.PriorityQ.put((h, new_node))
            self.bottom_view.append(new_node)
        self.bottom_view.remove(node)

    def draw_paths(self, win):
        for path in self.paths:
            path = path.ravel()
            for i in range(len(path)-1):
                pygame.draw.line(win, (255, 0, 0), path[i].pos, path[i+1].pos, width=4)


tree = Node([200, 200], radians(90), children=None, previous=None)


if __name__ == '__main__':
    start = None
    end = None
    run = False
    goal = Goal((700, 700), 40)
    pf = PathPlanner(tree, goal, dx=10, growth_rate=5, cone_angle=radians(60))
    res = 20
    iter = 0
    n_Q = len(pf.charges)
    fig, ax = plt.subplots()
    image = None
    exit_program = False
    while  not exit_program:
        t0 = time.time()
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    run = False
                    pygame.quit()
                    exit()
                    

                elif event.key == pygame.K_z: #positive
                    pos = pygame.mouse.get_pos()
                    pf.charges.append(Charge(pos, 1))

                elif event.key == pygame.K_x:
                    pos = pygame.mouse.get_pos()
                    pf.charges.append(Charge(pos, 0))

                elif event.key == pygame.K_c: # negative
                    pos = pygame.mouse.get_pos()
                    pf.charges.append(Charge(pos, -1))

                elif event.key == pygame.K_SPACE:
                    run = not run

                elif event.key == pygame.K_r:
                    tree = Node([200, 200], radians(90), children=None, previous=None)
                    pf.charges = []
                    pf.paths = []
        if run and len(pf.paths) < 10:
            iter += 1
            pf.grow_tree()
        WIN.fill(WHITE)
        pf.goal.draw_goal(WIN)
        for charge in pf.charges:
            charge.draw_charge(WIN)
        tree.draw(WIN)
        pf.draw_paths(WIN)
        pygame.display.update()
        try:
            f = f"Freq: {(time.time() - t0) ** -1:.2f}"
        except:
            f = "inf"
        print(f"Iteration num: {iter} {f}Hz Charges: {len(pf.charges)} Paths: {len(pf.paths)}")




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
        return d2goal*(np.exp(em_offset_theta/np.pi) + np.exp(O_offset_theta/np.pi) + 1)

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
            return self.calculate_curl(charges=charges) #+ 0.1*self.div_map(charges=charges)

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

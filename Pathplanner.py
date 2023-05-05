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

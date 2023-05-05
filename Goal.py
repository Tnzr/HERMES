
class Goal:  # Goal class
    def __init__(self, pos, radius):
        self.pos = pos
        self.radius = radius

    def draw_goal(self, win):  # Draw Goal
        pygame.draw.circle(win, ORANGE, self.pos, self.radius)
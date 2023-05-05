class Charge:  # Charge Class
    def __init__(self, pos, pol):
        self.pos = pos
        self.pol = pol
        self.radius = 5

    def draw_charge(self, win):  # Draw the Charges
        color = [RED, YELLOW, GREEN]
        pygame.draw.circle(win, color[self.pol+1], self.pos, self.radius)
        pygame.draw.circle(win, color[self.pol+1], self.pos, self.radius+20, 2)

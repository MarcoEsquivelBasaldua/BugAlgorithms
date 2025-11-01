import pygame

class Button:
    def __init__(self, x, y, width, height, text, color, pressedColor, wasPressed=False):
        self.rect = pygame.Rect(x, y, width, height)
        self.text = text
        self.color = color
        self.pressedColor = pressedColor
        self.currentColor = color
        self.font = pygame.font.SysFont("Arial", 24)
        self.wasPressed = wasPressed

    def draw(self, surface):
        pygame.draw.rect(surface, self.currentColor, self.rect)

        text_surface = self.font.render(self.text, True, (255, 255, 255)) # White text
        text_rect = text_surface.get_rect(center=self.rect.center)
        surface.blit(text_surface, text_rect)

    def handle_event(self, event):
        if self.rect.collidepoint(event.pos):
            self.wasPressed = True
            self.currentColor = self.pressedColor

    def reset(self):
        self.wasPressed = False
        self.currentColor = self.color



#class Visualizer:

#class SlideBar:
    
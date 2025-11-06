import pygame

class Button:
    def __init__(self, x, y, width, height, text, color, pressedColor, wasPressed=False):
        self.wasPressed     = wasPressed
        self.__rect         = pygame.Rect(x, y, width, height)
        self.__text         = text
        self.__color        = color
        self.__pressedColor = pressedColor
        self.__currentColor = color
        self.__font         = pygame.font.SysFont("Arial", 20)

    def draw(self, surface):
        pygame.draw.rect(surface, self.__currentColor, self.__rect)

        text_surface = self.__font.render(self.__text, True, (0, 0, 0)) # Black text
        text_rect    = text_surface.get_rect(center=self.__rect.center)

        surface.blit(text_surface, text_rect)

    def handle_event(self, event):
        if self.__rect.collidepoint(event.pos):
            self.wasPressed     = True
            self.__currentColor = self.__pressedColor

    def reset(self):
        self.wasPressed     = False
        self.__currentColor = self.__color



#class Visualizer:

#class SlideBar:
    
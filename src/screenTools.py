import pygame
import numpy as np

class Button:
    def __init__(self, x, y, width, height, text, color, pressedColor, screen, wasPressed=False):
        """
        Initializes a button with position, size, text, colors, and pressed state.
        Arguments:
            x: The x-coordinate of the button's top-left corner.
            y: The y-coordinate of the button's top-left corner.
            width: The width of the button.
            height: The height of the button.
            text: The text displayed on the button.
            color: The normal color of the button.
            pressedColor: The color of the button when pressed.
            screen: The pygame surface where the button will be drawn.
            wasPressed: A boolean indicating if the button was pressed.
        Returns:
            None
        """
        self.wasPressed     = wasPressed
        self.__rect         = pygame.Rect(x, y, width, height)
        self.__text         = text
        self.__color        = color
        self.__pressedColor = pressedColor
        self.__currentColor = color
        self.__screen       = screen
        self.__font         = pygame.font.SysFont("Arial", 20)

    def draw(self):
        """
        Draws the button on the given surface.
        Arguments:
            None
        Returns:
            None
        """
        pygame.draw.rect(self.__screen, self.__currentColor, self.__rect)

        text_surface = self.__font.render(self.__text, True, (0, 0, 0)) # Black text
        text_rect    = text_surface.get_rect(center=self.__rect.center)

        self.__screen.blit(text_surface, text_rect)

    def handle_event(self, event):
        """
        Handles mouse events to update the button's state.
        Arguments:
            event: The pygame event to handle.
        Returns:
            None
        """
        if self.__rect.collidepoint(event.pos):
            self.wasPressed     = True
            self.__currentColor = self.__pressedColor

    def reset(self):
        """
        Resets the button's pressed state and color.
        Arguments:
            None
        Returns:
            None
        """
        self.wasPressed     = False
        self.__currentColor = self.__color

    def was_button_pressed(self):
        """
        Returns whether the button was pressed.
        Arguments:
            None
        Returns:
            bool: True if the button was pressed, False otherwise.
        """
        return self.wasPressed



class Visualizer:
    def __init__(self, pos, fontSize, width, color, screen):
        """
        Initializes a visualizer with position, font size, width, color, and fixed message.
        Arguments:
            pos: A tuple (x, y) representing the position of the visualizer.
            fontSize: The font size for the text.
            width: The width of the visualizer.
            color: The color of the visualizer background.
            screen: The pygame surface where the visualizer will be drawn.
        Returns:
            None
        """
        self.pos      = pos
        self.color    = color
        self.height   = fontSize + 10
        self.width    = width
        self.__screen = screen
        self.__rect   = pygame.Rect(pos[0], pos[1], width, self.height)
        self.__font   = pygame.font.SysFont("Arial", fontSize)

    def draw(self, messageVar):
        """
        Draws the visualizer on the given screen with a variable message.
        Arguments:
            message: A variable message string to display alongside the fixed message.
        Returns:
            None
        """
        text_surface = self.__font.render(messageVar, True, self.color)
        text_rect    = text_surface.get_rect(center=self.__rect.center)

        self.__screen.blit(text_surface, text_rect)


class SlideBar:
    def __init__(self, pos, width, height, color, pressedColor, screen):
        """
        Initializes a slide bar with position, size, colors, and range.
        Arguments:
            pos: A tuple (x, y) representing the position of the slide bar.
            width: The width of the slide bar.
            height: The height of the slide bar.
            color: The normal color of the slide bar.
            pressedColor: The color of the slide bar when pressed.
            screen: The pygame surface where the slide bar will be drawn.
        Returns:
            None
        """
        self.range        = 0
        self.pos          = pos
        self.width        = width
        self.height       = height
        self.color        = color
        self.pressedColor = pressedColor
        self.__screen     = screen
        self.__sliderPos  = pos[0]
        self.__maxRange   = 100
        self.__rect       = pygame.Rect(pos[0], pos[1], width, self.height)

    def draw(self):
        """
        Draws the slide bar on the given screen.
        Arguments:
            screen: The pygame surface where the slide bar will be drawn.
        Returns:
            None
        """
        pygame.draw.rect(self.__screen, self.color, self.__rect)

        # Selector position
        pos = np.array((self.__sliderPos, self.pos[1] + self.height//2)).astype(int)
        pygame.draw.circle(self.__screen, self.pressedColor, pos, self.height//2 + 2)

    def handle_event(self, event):
        """
        Handles mouse events to update the slide bar's position and range.
        Arguments:
            event: The pygame event to handle.
        Returns:
            None
        """
        if self.__rect.collidepoint(event.pos):
            self.__sliderPos = event.pos[0]

            r = event.pos[0] - self.pos[0]

            if r < 1:
                self.range = 0
            else:
                ratio = r / self.width
                self.range = np.round(ratio * self.__maxRange).astype(int)
